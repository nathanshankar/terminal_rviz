#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/config_helper.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"
#include "terminal_rviz/displays/nav2_display.hpp"
#include "terminal_rviz/displays/motion_planning_display.hpp"
#include "terminal_rviz/displays/map_display.hpp"
#ifdef HAS_ROSBAG2
#include "terminal_rviz/displays/rosbag_display.hpp"
#endif
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <chrono>
#include <set>
#include <map>
#include <algorithm>
#include <sstream>
#include "ftxui/dom/canvas.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/event.hpp"

using namespace ftxui;
using namespace std::chrono_literals;

namespace terminal_rviz {

Visualizer::Visualizer(rclcpp::Node::SharedPtr node)
    : node_(node), screen_(ScreenInteractive::Fullscreen()) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

void Visualizer::add_display(std::shared_ptr<Display> display) {
    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
    displays_.push_back(display);
}

void Visualizer::cycle_tool() {
    if (current_tool_ == Tool::Nav2) current_tool_ = Tool::MotionPlanning;
    else if (current_tool_ == Tool::MotionPlanning) current_tool_ = Tool::Orbit;
    else if (current_tool_ == Tool::Orbit) current_tool_ = Tool::Pan;
    else current_tool_ = Tool::Nav2;
    last_mouse_x_ = 0; last_mouse_y_ = 0;
}

std::string Visualizer::get_tool_name() const {
    if (current_tool_ == Tool::Nav2) return "NAV2";
    if (current_tool_ == Tool::MotionPlanning) return "PLANNING";
    if (current_tool_ == Tool::Orbit) return "ORBIT";
    return "PAN";
}

void Visualizer::run() {
    auto main_renderer = Renderer([this]() { return render_frame(); });
    
    auto component = CatchEvent(main_renderer, [this](Event event) { 
        if (event.is_mouse()) {
            auto mouse = event.mouse();
            float dx = (last_mouse_x_ == 0) ? 0 : static_cast<float>(mouse.x - last_mouse_x_);
            float dy = (last_mouse_y_ == 0) ? 0 : static_cast<float>(mouse.y - last_mouse_y_);
            last_mouse_x_ = mouse.x; last_mouse_y_ = mouse.y;

            if (handle_event(event, static_cast<int>(dx))) return true;

            if (mouse.button == Mouse::WheelUp)   { 
                if (current_tool_ == Tool::MotionPlanning) {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (auto& d : displays_) if (d->getName() == "MotionPlanning" && d->isEnabled()) {
                        auto mp = std::dynamic_pointer_cast<MotionPlanningDisplay>(d);
                        if (mp && mp->handle_event(Event::Custom)) mp->update_goal_relative(0,0,0, 0,0, 0.1f); // Yaw
                        else if (mp) mp->update_goal_relative(0,0,0.1f, 0,0,0); // Z
                        screen_.PostEvent(Event::Custom); return true;
                    }
                }
                tar_zoom_ = tar_zoom_.load() * 1.1f; return true; 
            }
            if (mouse.button == Mouse::WheelDown) { 
                if (current_tool_ == Tool::MotionPlanning) {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (auto& d : displays_) if (d->getName() == "MotionPlanning" && d->isEnabled()) {
                        auto mp = std::dynamic_pointer_cast<MotionPlanningDisplay>(d);
                        if (mp && mp->handle_event(Event::Custom)) mp->update_goal_relative(0,0,0, 0,0,-0.1f); // Yaw
                        else if (mp) mp->update_goal_relative(0,0,-0.1f, 0,0,0); // Z
                        screen_.PostEvent(Event::Custom); return true;
                    }
                }
                tar_zoom_ = tar_zoom_.load() / 1.1f; return true; 
            }

            bool is_active = (mouse.motion == Mouse::Pressed);

            if (mouse.button == Mouse::Left) {
                if (current_tool_ == Tool::Orbit) {
                    if (is_active && std::abs(dx) < 50) { 
                        tar_yaw_ = tar_yaw_.load() + dx * 0.015f; 
                        tar_pitch_ = tar_pitch_.load() - dy * 0.015f; 
                    }
                } else if (current_tool_ == Tool::Pan) {
                    if (is_active && std::abs(dx) < 100) {
                        float y = cur_yaw_, p = cur_pitch_;
                        float sy = std::sin(y), cy = std::cos(y);
                        float sp = std::sin(p), cp = std::cos(p);
                        float factor = tar_dist_.load() / tar_zoom_.load();
                        float mx = (dx * sy - dy * sp * cy) * factor;
                        float my = (-dx * cy - dy * sp * sy) * factor;
                        float mz = (-dy * cp) * factor;
                        tar_cam_x_ = tar_cam_x_.load() + mx;
                        tar_cam_y_ = tar_cam_y_.load() + my;
                        tar_cam_z_ = tar_cam_z_.load() + mz;
                    }
                } else if (current_tool_ == Tool::Nav2) {
                    std::shared_ptr<Nav2Display> nav2 = nullptr;
                    {
                        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                        for (auto& d : displays_) if (d->getName() == "Nav2") { nav2 = std::dynamic_pointer_cast<Nav2Display>(d); break; }
                    }
                    if (nav2 && nav2->isEnabled()) {
                        int vx = mouse.x - canvas_x_offset_;
                        int vy = mouse.y - canvas_y_offset_;
                        if (vx >= 0 && vy >= 0) {
                            float wx, wy; bool hit = renderer_.pick_ground_plane(vx * 2, vy * 4, wx, wy);
                            if (mouse.motion == Mouse::Pressed && hit) {
                                if (!nav2->is_selecting()) { nav2->set_goal(wx, wy, fixed_frame_); nav2->start_selection(); }
                                else nav2->update_goal_orientation(wx, wy);
                            } else if (mouse.motion == Mouse::Released) {
                                if (hit) nav2->update_goal_orientation(wx, wy);
                                nav2->finalize_selection();
                            }
                        }
                    }
                } else if (current_tool_ == Tool::MotionPlanning) {
                    std::shared_ptr<MotionPlanningDisplay> mp = nullptr;
                    {
                        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                        for (auto& d : displays_) if (d->getName() == "MotionPlanning") { mp = std::dynamic_pointer_cast<MotionPlanningDisplay>(d); break; }
                    }
                    if (mp && mp->isEnabled()) {
                        int vx = mouse.x - canvas_x_offset_, vy = mouse.y - canvas_y_offset_;
                        if (vx >= 0 && vy >= 0) {
                            if (mouse.motion == Mouse::Pressed) {
                                if (!mp->is_selecting()) {
                                    if (mp->is_hit(vx, vy, renderer_)) {
                                        mp->start_selection();
                                    }
                                } else {
                                    float factor = tar_dist_.load() / tar_zoom_.load() * 1.0f; // Increased sensitivity
                                    float y = cur_yaw_, sy = std::sin(y), cy = std::cos(y);
                                    if (mp->handle_event(Event::Custom)) { 
                                        mp->update_goal_relative(0,0,0, dy*0.05f, dx*0.05f, 0);
                                    } else {
                                        mp->update_goal_relative((dx*sy - dy*cy)*factor, (-dx*cy - dy*sy)*factor, 0, 0,0,0);
                                    }
                                }
                            } else if (mouse.motion == Mouse::Released) {
                                mp->finalize_selection();
                            }
                        }
                    }
                }
                return true;
            }
            if (mouse.button == Mouse::Right && is_active) {
                tar_yaw_ = tar_yaw_.load() + dx * 0.015f; tar_pitch_ = tar_pitch_.load() - dy * 0.015f; return true;
            }
            if (mouse.button == Mouse::Middle && is_active) {
                float f = tar_dist_.load() * 0.005f, y = cur_yaw_;
                tar_cam_x_ = tar_cam_x_.load() - (dy * 2.5f * std::cos(y) + dx * std::sin(y)) * f;
                tar_cam_y_ = tar_cam_y_.load() - (dy * 2.5f * std::sin(y) - dx * std::cos(y)) * f;
                return true;
            }
            return true;
        }

        if (handle_event(event)) return true;
        return false;
    });

    std::thread ui_thread([&]() {
        while (!quit_flag_ && rclcpp::ok()) { discover_frames(); discover_topics(); screen_.PostEvent(Event::Custom); std::this_thread::sleep_for(50ms); }
        screen_.Exit();
    });

    screen_.Loop(component);
    quit_flag_ = true; if (ui_thread.joinable()) ui_thread.join();
}

void Visualizer::stop() { quit_flag_ = true; screen_.Exit(); }

void Visualizer::refresh_file_list() {
    file_list_.clear();
    try {
        if (!std::filesystem::exists(current_path_)) current_path_ = std::filesystem::current_path();
        
        std::filesystem::path dir_to_read = current_path_;
        if (!std::filesystem::is_directory(dir_to_read)) dir_to_read = dir_to_read.parent_path();

        for (const auto& entry : std::filesystem::directory_iterator(dir_to_read)) {
            if (!is_save_mode_) {
                if (!entry.is_directory() && entry.path().extension() != ".nathan") continue;
            }
            file_list_.push_back(entry);
        }
        std::sort(file_list_.begin(), file_list_.end(), [](const auto& a, const auto& b) {
            if (a.is_directory() != b.is_directory()) return a.is_directory();
            return a.path().filename().string() < b.path().filename().string();
        });
    } catch (...) {}
}

void Visualizer::save_config(const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) { status_msg_ = "Failed to open file for saving"; return; }

    f << "fixed_frame \"" << fixed_frame_ << "\"\n";
    f << "camera " << tar_yaw_.load() << " " << tar_pitch_.load() << " " << tar_dist_.load() << " "
      << tar_cam_x_.load() << " " << tar_cam_y_.load() << " " << tar_cam_z_.load() << " " << tar_zoom_.load() << "\n";
    
    if (grid_display_) {
        f << "grid " << (grid_display_->isEnabled() ? 1 : 0) << "\n";
    }

    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
    for (auto& d : displays_) {
        f << "display \"" << d->getName() << "\" " << (d->isAdded() ? 1 : 0) << " " << (d->isEnabled() ? 1 : 0) << " \"" << d->getTopic() << "\"\n";
        auto topics = d->getEnabledTopics();
        // Ensure the primary topic is included if it's not already
        if (!d->getTopic().empty() && d->getTopic() != "None") {
            if (std::find(topics.begin(), topics.end(), d->getTopic()) == topics.end()) {
                topics.push_back(d->getTopic());
            }
        }
        for (const auto& t : topics) {
            auto cfg = d->getTopicConfig(t);
            f << "config \"" << d->getName() << "\" \"" << t << "\" " << cfg.alpha << " " << cfg.size << " " 
              << (int)cfg.r << " " << (int)cfg.g << " " << (int)cfg.b << " \"" << cfg.color_style << "\" " 
              << cfg.color_index << " " << cfg.color_index_2 << " \"" << cfg.axis << "\" \"" << cfg.style << "\" " << cfg.history_length << "\n";
        }
    }
    status_msg_ = "Config saved to " + path;
}

static std::string read_quoted(std::istream& is) {
    std::string s;
    is >> std::ws;
    if (is.peek() == '"') {
        is.get(); // skip opening quote
        std::getline(is, s, '"');
    } else {
        is >> s;
    }
    return s;
}

void Visualizer::load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) { status_msg_ = "Failed to open file for loading"; return; }

    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        modal_topic_selections_.clear();
        for (auto& d : displays_) {
            d->setAdded(false);
            d->setEnabled(false);
        }
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string type; ss >> type;
        if (type == "fixed_frame") {
            fixed_frame_ = read_quoted(ss);
            auto it = std::find(available_frames_.begin(), available_frames_.end(), fixed_frame_);
            if (it != available_frames_.end()) frame_idx_ = std::distance(available_frames_.begin(), it);
        } else if (type == "camera") {
            float y, p, d, cx, cy, cz, z;
            ss >> y >> p >> d >> cx >> cy >> cz >> z;
            tar_yaw_ = y; tar_pitch_ = p; tar_dist_ = d;
            tar_cam_x_ = cx; tar_cam_y_ = cy; tar_cam_z_ = cz; tar_zoom_ = z;
        } else if (type == "grid") {
            int enabled; ss >> enabled;
            if (grid_display_) grid_display_->setEnabled(enabled);
        } else if (type == "display") {
            std::string name = read_quoted(ss);
            int added, enabled;
            ss >> added >> enabled;
            std::string topic = read_quoted(ss);
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (auto& d : displays_) {
                if (d->getName() == name) {
                    d->setAdded(added);
                    d->setEnabled(enabled);
                    
                    // Keep modal states in sync
                    int d_idx = -1;
                    for (size_t i = 0; i < displays_.size(); ++i) {
                        if (displays_[i] == d) { d_idx = (int)i; break; }
                    }
                    if (d_idx >= 0 && d_idx < 64) {
                        modal_plugin_states_[d_idx] = added;
                    }

                    // PointCloud2 and similar might have multiple topics, but setTopic usually adds/toggles.
                    // For single-topic displays, this is straightforward.
                    if (!topic.empty() && topic != "None") {
                        if (d->getTopic() != topic) d->setTopic(topic);
                        
                        // Keep modal selections in sync
                        int d_idx = -1;
                        for (size_t i = 0; i < displays_.size(); ++i) {
                            if (displays_[i] == d) { d_idx = (int)i; break; }
                        }
                        if (d_idx >= 0) {
                            modal_topic_selections_[{topic, d_idx}] = true;
                        }
                    }
                    break;
                }
            }
        } else if (type == "config") {
            std::string dname = read_quoted(ss);
            std::string tname = read_quoted(ss);
            TopicConfig cfg;
            int r, g, b;
            ss >> cfg.alpha >> cfg.size >> r >> g >> b;
            cfg.r = (uint8_t)r; cfg.g = (uint8_t)g; cfg.b = (uint8_t)b;
            cfg.color_style = read_quoted(ss);
            ss >> cfg.color_index >> cfg.color_index_2;
            cfg.axis = read_quoted(ss);
            cfg.style = read_quoted(ss);
            ss >> cfg.history_length;
            
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (auto& d : displays_) {
                if (d->getName() == dname) {
                    // If it's a multi-topic display, we might need to ensure the topic is enabled first
                    if (!d->isTopicEnabled(tname) && tname != "None" && !tname.empty()) {
                        d->setTopic(tname);
                    }
                    d->setTopicConfig(tname, cfg);
                    
                    // Keep modal selections in sync
                    if (tname != "None" && !tname.empty()) {
                        int d_idx = -1;
                        for (size_t i = 0; i < displays_.size(); ++i) {
                            if (displays_[i] == d) { d_idx = (int)i; break; }
                        }
                        if (d_idx >= 0) {
                            modal_topic_selections_[{tname, d_idx}] = true;
                        }
                    }
                    break;
                }
            }
        }
    }
    status_msg_ = "Config loaded from " + path;
    discover_topics();
}

void Visualizer::discover_frames() {
    if (!tf_buffer_) return;
    std::vector<std::string> frames; tf_buffer_->_getFrameStrings(frames);
    if (frames.empty()) return;
    std::set<std::string> frame_set(frames.begin(), frames.end());
    std::vector<std::string> sorted_frames(frame_set.begin(), frame_set.end());
    if (sorted_frames != available_frames_) {
        std::string current = available_frames_.empty() ? "" : available_frames_[frame_idx_];
        available_frames_ = sorted_frames;
        auto it = std::find(available_frames_.begin(), available_frames_.end(), current);
        frame_idx_ = (it != available_frames_.end()) ? std::distance(available_frames_.begin(), it) : 0;
    }
    if (!available_frames_.empty()) fixed_frame_ = available_frames_[frame_idx_];
}

void Visualizer::build_topic_tree() {
    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
    modal_topic_entries_.clear();
    
    // We don't clear modal_topic_selections_ here as they represent the state of what's added

    struct Node {
        std::string name;
        std::string full_path;
        std::map<std::string, std::unique_ptr<Node>> children;
        std::vector<int> matching_plugins;
    };

    Node root{"", "", {}, {}};

    auto topic_map = node_->get_topic_names_and_types();
    for (const auto& [topic_name, types] : topic_map) {
        for (const auto& type : types) {
            std::vector<int> matches;
            for (size_t i = 0; i < displays_.size(); ++i) {
                if (displays_[i]->getMessageType() == type) {
                    matches.push_back(i);
                }
            }

            if (!matches.empty()) {
                std::stringstream ss(topic_name);
                std::string segment;
                Node* current = &root;
                std::string path = "";
                
                while (std::getline(ss, segment, '/')) {
                    if (segment.empty()) continue;
                    path += "/" + segment;
                    if (current->children.find(segment) == current->children.end()) {
                        current->children[segment] = std::make_unique<Node>(Node{segment, path, {}, {}});
                    }
                    current = current->children[segment].get();
                }
                for (int idx : matches) current->matching_plugins.push_back(idx);
            }
        }
    }

    std::function<void(Node*, int)> flatten = [&](Node* n, int indent) {
        bool expanded = false;
        if (!n->name.empty()) {
            expanded = (expanded_topic_nodes_.find(n->full_path) != expanded_topic_nodes_.end());
            bool has_children = !n->children.empty() || !n->matching_plugins.empty();
            modal_topic_entries_.push_back({n->name, n->full_path, -1, indent, false, expanded, has_children});
        }
        
        if (n->name.empty() || expanded) {
            for (auto& [name, child] : n->children) {
                flatten(child.get(), n->name.empty() ? 0 : indent + 1);
            }

            for (int p_idx : n->matching_plugins) {
                modal_topic_entries_.push_back({displays_[p_idx]->getName(), n->full_path, p_idx, indent + 1, true, false, false});
            }
        }
    };

    flatten(&root, 0);
}

void Visualizer::discover_topics() {
    auto topic_map = node_->get_topic_names_and_types();
    std::vector<std::string> topics;
    
    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
        auto disp = displays_[plugin_idx_];
        std::string target_type = disp->getMessageType();
        if (target_type == "TF") {
            if (tf_buffer_) tf_buffer_->_getFrameStrings(topics);
        } else if (target_type != "None") {
            for (const auto& [name, types] : topic_map) {
                for (const auto& type : types) {
                    if (type == target_type) {
                        topics.push_back(name);
                        break;
                    }
                }
            }
        }
    }
    std::sort(topics.begin(), topics.end());
    if (topics != available_topics_) {
        std::string current = (available_topics_.empty() || topic_idx_ >= (int)available_topics_.size()) ? "" : available_topics_[topic_idx_];
        available_topics_ = topics;
        auto it = std::find(available_topics_.begin(), available_topics_.end(), current);
        topic_idx_ = (it != available_topics_.end()) ? std::distance(available_topics_.begin(), it) : 0;
    }
}

Element Visualizer::render_frame() {
    auto terminal = ftxui::Terminal::Size();
    Element image_panel = filler(), nav2_panel = filler(), teleop_panel = filler(), rosbag_panel = filler(), config_panel = filler();
    bool nav2_active = false;
    bool teleop_active = false;
#ifdef HAS_ROSBAG2
    bool rosbag_active = false;
#endif

    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Nav2" && d->isAdded() && d->isEnabled()) nav2_active = true;
            if (d->getName() == "Teleop" && d->isAdded() && d->isEnabled()) teleop_active = true;
#ifdef HAS_ROSBAG2
            if (d->getName() == "Rosbag" && d->isAdded() && d->isEnabled()) rosbag_active = true;
#endif
        }
    }

    bool show_blocking_modal = show_plugin_modal_ || show_frame_modal_ || show_file_modal_;

    int left_width = show_blocking_modal ? 0 : 30;
    int right_width = 0;

    bool has_right_content = false;
    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& display : displays_) {
            if (!display->isAdded() || !display->isEnabled()) continue;
            if (display->getName() == "Image") {
                auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(display);
                if (img_disp && img_disp->getEnabledTopicCount() > 0) { 
                    has_right_content = true;
                    if (!show_blocking_modal) image_panel = img_disp->render_2d(nav2_active || teleop_active);
                }
            }
        }
    }
    if (nav2_active) {
        has_right_content = true;
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Nav2") {
                if (!show_blocking_modal) nav2_panel = d->render_2d();
                break;
            }
        }
    }

    if (teleop_active) {
        has_right_content = true;
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Teleop") {
                if (!show_blocking_modal) teleop_panel = d->render_2d();
                break;
            }
        }
    }

#ifdef HAS_ROSBAG2
    if (rosbag_active) {
        has_right_content = true;
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Rosbag") {
                if (!show_blocking_modal) rosbag_panel = d->render_2d(nav2_active || teleop_active, config_scroll_);
                break;
            }
        }
    }
#endif

    bool config_active = false;
    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            auto disp = displays_[plugin_idx_];
            if (disp->getName() != "Image" && disp->getName() != "Nav2" && disp->getName() != "Teleop" && disp->getName() != "Rosbag" && disp->isAdded() && disp->isEnabled()) {
                has_right_content = true;
                if (!show_blocking_modal) {
                    config_panel = disp->render_2d(nav2_active || teleop_active 
#ifdef HAS_ROSBAG2
                        || rosbag_active
#endif
                        , config_scroll_);
                    config_active = true;
                }
            }
        }
    }

    if (!show_blocking_modal && has_right_content) right_width = 64;
    if (!show_blocking_modal && terminal.dimx < 120) right_width = std::min(right_width, terminal.dimx / 3);

    const int target_height = std::max(10, terminal.dimy - 8);
    const int target_width = std::max(10, terminal.dimx - left_width - right_width - 6);
    right_width_ = right_width;
    const int sw = target_width * 2, sh = target_height * 4;
    
    auto c = Canvas(sw, sh);
    
    const float alpha = 0.2f;
    cur_yaw_   += (tar_yaw_.load() - cur_yaw_) * alpha;
    cur_pitch_ += (tar_pitch_.load() - cur_pitch_) * alpha;
    cur_dist_  += (tar_dist_.load() - cur_dist_) * alpha;
    cur_cam_x_ += (tar_cam_x_.load() - cur_cam_x_) * alpha;
    cur_cam_y_ += (tar_cam_y_.load() - cur_cam_y_) * alpha;
    cur_cam_z_ += (tar_cam_z_.load() - cur_cam_z_) * alpha;
    cur_zoom_  += (tar_zoom_.load() - cur_zoom_) * alpha;

    renderer_.set_size(sw, sh);
    renderer_.set_camera(cur_yaw_, cur_pitch_, 0.0f, cur_dist_, cur_cam_x_, cur_cam_y_, cur_cam_z_);
    renderer_.set_zoom(cur_zoom_);
    renderer_.enable_gpu(use_gpu_);
    
    renderer_.clear(); // Always clear to prevent stale labels/grid lines
    if (!show_blocking_modal) {
        // Due to "first-come-first-served" Z-buffering (z < dot.z), 
        // we render high-priority items FIRST so they aren't overdrawn by Map/PointCloud.
        
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        
        // 1. Render Path and other high-priority plugins (Top layer)
        for (auto& display : displays_) {
            if (display->isAdded() && display->isEnabled() && 
                display->getName() != "PointCloud2" && display->getName() != "Map") {
                display->render(renderer_, c, fixed_frame_, tf_buffer_);
            }
        }

        // 2. Render Map (Middle layer)
        for (auto& display : displays_) {
            if (display->isAdded() && display->isEnabled() && display->getName() == "Map") {
                display->render(renderer_, c, fixed_frame_, tf_buffer_);
            }
        }

        // 3. Render PointClouds (Bottom layer)
        for (auto& display : displays_) {
            if (display->isAdded() && display->isEnabled() && display->getName() == "PointCloud2") {
                display->render(renderer_, c, fixed_frame_, tf_buffer_);
            }
        }

        if (grid_display_) grid_display_->render(renderer_, c, fixed_frame_, tf_buffer_);
        
        renderer_.finish(c);
    }

    canvas_x_offset_ = left_width + 2;
    canvas_y_offset_ = 4;

    Elements display_list;
    int visible_plugin_count = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (size_t i = 0; i < displays_.size(); ++i) {
            if (!displays_[i]->isAdded()) continue;
            
            if (visible_plugin_count >= plugin_scroll_ && visible_plugin_count < plugin_scroll_ + 8) {
                bool selected = (static_cast<int>(i) == plugin_idx_);
                bool hovered = (static_cast<int>(i) == sidebar_hover_idx_);
                bool enabled = displays_[i]->isEnabled();
                auto t = text(" " + displays_[i]->getName());
                
                if (selected) {
                    t = t | inverted | color(enabled ? Color::Yellow : Color::RedLight) | focus;
                } else if (hovered) {
                    t = t | color(enabled ? Color::Green : Color::Red) | bgcolor(Color::GrayDark);
                } else {
                    t = t | color(enabled ? Color::Green : Color::Red);
                }
                display_list.push_back(t);
            }
            visible_plugin_count++;
        }
    }
    if (display_list.empty()) {
        display_list.push_back(text(" No plugins added") | dim);
        display_list.push_back(text(" Press [P] to add") | dim);
    }

    Elements topic_list;
    int visible_topic_count = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            auto disp = displays_[plugin_idx_];
            std::string type = disp->getMessageType();
            if (type == "None") topic_list.push_back(text("No settings") | dim);
            else {
                std::string label = (type == "TF") ? "Frames [Y/H]:" : ("Type: " + type);
                topic_list.push_back(text(label) | dim | size(WIDTH, LESS_THAN, 25));
                topic_list.push_back(separator());
                
                int max_visible_topics = std::max(5, target_height - 25); 

                for (size_t i = 0; i < available_topics_.size(); ++i) {
                    if (visible_topic_count >= topic_scroll_ && visible_topic_count < topic_scroll_ + max_visible_topics) {
                        bool is_selected = (static_cast<int>(i) == topic_idx_);
                        auto name = available_topics_[i];
                        if (type == "TF") {
                            auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(disp);
                            bool enabled = tf_disp && tf_disp->isFrameEnabled(name);
                            auto t = text((enabled ? "[X] " : "[ ] ") + name);
                            topic_list.push_back(is_selected ? (t | inverted | color(Color::Magenta) | focus) : t);
                        } else if (type == "sensor_msgs/msg/Image") {
                            auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(disp);
                            bool enabled = img_disp && img_disp->isTopicEnabled(name);
                            auto t = text(" " + name);
                            if (enabled) t = t | color(Color::Green);
                            else t = t | color(Color::Red);
                            if (is_selected) t = t | inverted | focus;
                            topic_list.push_back(t);
                        } else {
                            bool enabled = disp->isTopicEnabled(name);
                            auto t = text(" " + name);
                            if (enabled) t = t | color(Color::Green);
                            else t = t | color(Color::Red);
                            if (is_selected) t = t | inverted | focus;
                            topic_list.push_back(t);
                        }                }
                    visible_topic_count++;
                }
            }
        } else topic_list.push_back(text("Select a plugin (Tab)") | dim);
    }

    auto main_layout = vbox({
        hbox({
            text(" TERMINAL RVIZ ") | bold | color(Color::Yellow),
            separator(),
            text(" Frame: " + fixed_frame_) | color(Color::Cyan),
            separator(),
            text(" [SAVE] ") | (is_save_mode_ && show_file_modal_ ? inverted : nothing) | color(Color::Green),
            text(" [LOAD] ") | (!is_save_mode_ && show_file_modal_ ? inverted : nothing) | color(Color::Cyan),
            filler(),
            text(" [P] Plugins | [F] Frame | [V] Tool | [R] Reset | [T] TopDown | [G] Grid | [Tab] Select | [Space] Toggle | [Esc] Quit ") | dim
        }),
        separator(),
        hbox({
            vbox({
                vbox({
                    text(" FIXED FRAME [F] ") | bold | color(Color::Yellow),
                    text(" " + fixed_frame_) | color(Color::Cyan) | border,
                }) | border,
                vbox({
                    text(" ACTIVE TOOL [V] ") | bold | color(Color::Yellow),
                    text(" " + get_tool_name()) | color(Color::White) | border | hcenter,
                }) | border,
                vbox({
                    text(" PLUGINS [P] ") | bold | color(Color::Yellow),
                    vbox(std::move(display_list)) | border | size(HEIGHT, EQUAL, 8),
                }) | border,
                vbox({
                    text(" TOPICS/FRAMES [Y/H] ") | bold | color(Color::Yellow),
                    vbox(std::move(topic_list)) | border | flex,
                }) | border | flex,
                vbox({
                    text(" STATUS: ") | bold,
                    text(status_msg_) | color(Color::Green),
                }) | border | size(HEIGHT, EQUAL, 4),
                }) | size(WIDTH, EQUAL, left_width),
                [&] {
                Elements label_elements;
                label_elements.push_back(canvas(std::move(c)) | flex);

                for (const auto& label : renderer_.get_labels()) {
                    int sx, sy; float sz;
                    if (renderer_.project(label.x, label.y, label.z, sx, sy, sz)) {
                        int tx = sx / 2;
                        int ty = sy / 4;
                        if (tx >= 0 && tx < target_width && ty >= 0 && ty < target_height) {
                            label_elements.push_back(
                                vbox({
                                    filler() | size(HEIGHT, EQUAL, ty),
                                    hbox({
                                        filler() | size(WIDTH, EQUAL, tx),
                                        text(label.text) | color(label.color),
                                        filler()
                                    }),
                                    filler()
                                })
                            );
                        }
                    }
                }
                auto view = dbox(std::move(label_elements)) | center | flex | border;
                if (show_blocking_modal) view = view | color(Color::Black) | bgcolor(Color::Black);
                return view;
                }(),
                vbox({ 
                (show_topic_modal_ || show_config_modal_ ? (filler() | bgcolor(Color::Black)) : image_panel) | flex, 
                (show_topic_modal_ || show_config_modal_ ? (filler() | size(HEIGHT, EQUAL, 14) | bgcolor(Color::Black)) : 
                    (config_active ? config_panel : 
#ifdef HAS_ROSBAG2
                    (rosbag_active ? rosbag_panel : 
#endif
                    (teleop_active ? teleop_panel : (nav2_active ? nav2_panel : filler()))
#ifdef HAS_ROSBAG2
                    )
#endif
                    )) 
                }) | size(WIDTH, EQUAL, right_width),
                }) | flex
                });
    Element modal_box = filler();

    if (show_file_modal_) {
        Elements items;
        int max_visible = 15;
        int count = (int)file_list_.size();
        
        int visible_start = std::max(0, file_scroll_);
        int visible_end = std::min(count, visible_start + max_visible);

        auto up_one = text(" [ .. ] ") | color(Color::Yellow);
        if (file_selected_idx_ == -1) up_one = up_one | inverted | focus;
        items.push_back(up_one);

        for (int i = visible_start; i < visible_end; ++i) {
            bool selected = (i == file_selected_idx_);
            auto name = file_list_[i].path().filename().string();
            if (file_list_[i].is_directory()) name = "[D] " + name;
            auto t = text(" " + name);
            if (selected) t = t | inverted | focus;
            else if (file_list_[i].is_directory()) t = t | color(Color::Blue);
            else if (file_list_[i].path().extension() == ".nathan") t = t | color(Color::Green);
            items.push_back(t);
        }

        auto save_load_btn = text(is_dir_picker_ ? " [ SELECT ] " : " [ SAVE ] ");
        if (file_selected_idx_ == count) save_load_btn = save_load_btn | inverted | color(Color::Green) | focus;

        auto cancel_btn = text(" [ CANCEL ] ");
        if (file_selected_idx_ == count + 1) cancel_btn = cancel_btn | inverted | color(Color::Red) | focus;

        Elements button_row;
        button_row.push_back(filler());
        if (is_save_mode_ || is_dir_picker_) button_row.push_back(save_load_btn);
        button_row.push_back(cancel_btn);
        button_row.push_back(filler());

        int current_modal_h = is_save_mode_ ? 27 : 24;
        std::string title = is_save_mode_ ? " SAVE CONFIG " : (is_dir_picker_ ? " SELECT DIRECTORY " : " LOAD CONFIG ");
        Color title_color = is_save_mode_ ? Color::Green : (is_dir_picker_ ? Color::Yellow : Color::Cyan);

        Elements modal_content;
        modal_content.push_back(text(title) | bold | hcenter | color(title_color));
        modal_content.push_back(text(" Path: " + current_path_.string()) | dim | size(WIDTH, EQUAL, 60));
        modal_content.push_back(separator());
        modal_content.push_back(vbox(std::move(items)) | size(HEIGHT, EQUAL, max_visible + 1) | border);
        if (is_save_mode_) {
            modal_content.push_back(hbox({ text(" Filename: ") | vcenter, text(input_filename_ + "_") | border | color(Color::Yellow) }));
        }
        modal_content.push_back(hbox(std::move(button_row)));

        modal_box = dbox({
            filler() | bgcolor(Color::Black),
            vbox(std::move(modal_content)) | border
        }) | size(WIDTH, EQUAL, 64) | size(HEIGHT, EQUAL, current_modal_h) | center;
    } else if (show_plugin_modal_) {
        Elements modal_items;
        int max_visible = 15;
        int max_h = 15;
        
        std::vector<int> filtered_indices;
        {
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (size_t i = 0; i < displays_.size(); ++i) {
                bool is_panel = (displays_[i]->getName() == "Nav2" || displays_[i]->getName() == "Teleop" || displays_[i]->getName() == "Rosbag" || displays_[i]->getName() == "MotionPlanning");
                if (modal_tab_idx_ == 0 && !is_panel) filtered_indices.push_back(i);
                else if (modal_tab_idx_ == 1 && is_panel) filtered_indices.push_back(i);
            }
        }

        int display_count = (int)filtered_indices.size();
        
        if (modal_tab_idx_ == 2) {
            display_count = (int)modal_topic_entries_.size();
            if (modal_selected_idx_ < display_count) {
                if (modal_selected_idx_ < modal_scroll_) modal_scroll_ = modal_selected_idx_;
                if (modal_selected_idx_ >= modal_scroll_ + max_visible) modal_scroll_ = modal_selected_idx_ - max_visible + 1;
            }

            for (int i = modal_scroll_; i < std::min(display_count, modal_scroll_ + max_visible); ++i) {
                const auto& entry = modal_topic_entries_[i];
                std::string indent_str = "";
                for(int s=0; s<entry.indent; ++s) indent_str += "  ";
                
                std::string prefix = "";
                if (!entry.is_plugin_type) {
                    prefix = entry.is_expanded ? "▼ " : "▶ ";
                }
                
                std::string label = indent_str + prefix + entry.label;
                if (entry.is_plugin_type) {
                    bool checked = modal_topic_selections_[{entry.topic, entry.display_idx}];
                    label = indent_str + (checked ? "[X] " : "[ ] ") + entry.label;
                }

                auto t = text(label);
                if (i == modal_selected_idx_) t = t | inverted | focus;
                else t = t | color(entry.is_plugin_type ? Color::Cyan : Color::White);
                modal_items.push_back(t);
            }
        } else {
            if (modal_selected_idx_ < display_count) {
                if (modal_selected_idx_ < modal_scroll_) modal_scroll_ = modal_selected_idx_;
                if (modal_selected_idx_ >= modal_scroll_ + max_visible) modal_scroll_ = modal_selected_idx_ - max_visible + 1;
            }

            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (int i = modal_scroll_; i < std::min(display_count, modal_scroll_ + max_visible); ++i) {
                int original_idx = filtered_indices[i];
                bool selected = (i == modal_selected_idx_);
                bool checked = modal_plugin_states_[original_idx];
                auto t = text((checked ? " [X] " : " [ ] ") + displays_[original_idx]->getName());
                if (selected) t = t | inverted | focus;
                else t = t | color(checked ? Color::Green : Color::GrayDark);
                modal_items.push_back(t);
            }
        }

        auto tab_plugin = text(" PLUGINS ");
        auto tab_panel = text("  PANELS ");
        auto tab_topic = text("  TOPICS ");
        if (modal_tab_idx_ == 0) tab_plugin = tab_plugin | inverted | color(Color::Yellow) | bold;
        else if (modal_tab_idx_ == 1) tab_panel = tab_panel | inverted | color(Color::Yellow) | bold;
        else tab_topic = tab_topic | inverted | color(Color::Yellow) | bold;

        modal_box = vbox({
            text(" ADD DISPLAY ") | bold | hcenter | color(Color::Yellow),
            hbox({
                filler(),
                tab_plugin,
                separator(),
                tab_panel,
                separator(),
                tab_topic,
                filler(),
            }) | border,
            vbox(std::move(modal_items)) | size(HEIGHT, EQUAL, max_h) | border,
            hbox({
                filler(),
                text(" [ OK ] ") | (modal_tab_idx_ < 2 && modal_selected_idx_ == display_count ? (inverted | color(Color::Green) | focus) : nothing),
                text(" [ CANCEL ] ") | (modal_selected_idx_ == (modal_tab_idx_ == 2 ? (int)modal_topic_entries_.size() : display_count) + 1 ? (inverted | color(Color::Red) | focus) : nothing),
                filler(),
            })
        }) | size(WIDTH, EQUAL, 50) | border | bgcolor(Color::Black) | center;
    } else if (show_frame_modal_) {
        Elements modal_items;
        int max_visible = 15;
        int frame_count = (int)available_frames_.size();

        if (modal_frame_selected_idx_ < frame_count) {
            if (modal_frame_selected_idx_ < modal_frame_scroll_) modal_frame_scroll_ = modal_frame_selected_idx_;
            if (modal_frame_selected_idx_ >= modal_frame_scroll_ + max_visible) modal_frame_scroll_ = modal_frame_selected_idx_ - max_visible + 1;
        }

        for (int i = modal_frame_scroll_; i < std::min(frame_count, modal_frame_scroll_ + max_visible); ++i) {
            bool selected = (i == modal_frame_selected_idx_);
            bool current = (i == frame_idx_);
            auto t = text((current ? " > " : "   ") + available_frames_[i]);
            if (selected) t = t | inverted | focus;
            else if (current) t = t | color(Color::Cyan);
            modal_items.push_back(t);
        }

        modal_box = vbox({
            text(" FIXED FRAME SELECTION ") | bold | hcenter | color(Color::Cyan),
            separator(),
            vbox(std::move(modal_items)) | size(HEIGHT, EQUAL, max_visible) | border,
            hbox({
                filler(),
                text(" [ OK ] ") | (modal_frame_selected_idx_ == frame_count ? (inverted | color(Color::Green) | focus) : nothing),
                text(" [ CANCEL ] ") | (modal_frame_selected_idx_ == frame_count + 1 ? (inverted | color(Color::Red) | focus) : nothing),
                filler(),
            })
        }) | size(WIDTH, GREATER_THAN, 40) | border | bgcolor(Color::Black) | center;
    } else if (show_topic_modal_) {
        Elements modal_items;
        int max_visible = 15;
        int topic_count = (int)topic_modal_list_.size();

        if (topic_modal_selected_idx_ < topic_count) {
            if (topic_modal_selected_idx_ < topic_modal_scroll_) topic_modal_scroll_ = topic_modal_selected_idx_;
            if (topic_modal_selected_idx_ >= topic_modal_scroll_ + max_visible) topic_modal_scroll_ = topic_modal_selected_idx_ - max_visible + 1;
        }

        for (int i = topic_modal_scroll_; i < std::min(topic_count, topic_modal_scroll_ + max_visible); ++i) {
            bool selected = (i == topic_modal_selected_idx_);
            auto t = text(" " + topic_modal_list_[i]);
            if (selected) t = t | inverted | focus;
            modal_items.push_back(t);
        }

        modal_box = vbox({
            text(" SELECT TOPIC ") | bold | hcenter | color(Color::Magenta),
            separator(),
            vbox(std::move(modal_items)) | size(HEIGHT, EQUAL, max_visible) | border,
            hbox({
                filler(),
                text(" [ CANCEL ] ") | (topic_modal_selected_idx_ == topic_count ? (inverted | color(Color::Red) | focus) : nothing),
                filler(),
            })
        }) | size(WIDTH, EQUAL, 60) | border | bgcolor(Color::Black);

        int modal_w = 60;
        int items_h = std::min(15, (int)topic_modal_list_.size());
        int modal_h = items_h + 7;
        
        int x = std::clamp(topic_modal_x_ - modal_w / 2, 0, terminal.dimx - modal_w);
        int y = std::clamp(topic_modal_y_ - modal_h / 2, 0, terminal.dimy - modal_h);
        
        modal_box = vbox({
            filler() | size(HEIGHT, EQUAL, y),
            hbox({
                filler() | size(WIDTH, EQUAL, x),
                modal_box,
                filler(),
            }),
            filler(),
        });
    } else if (show_config_modal_) {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        auto cfg = config_target_display_->getTopicConfig(config_modal_topic_);
        modal_box = ConfigHelper::render_edit_modal(config_modal_topic_, cfg, 
                                                   config_modal_selected_idx_, right_width_,
                                                   config_target_display_->getName());
    }

    if (!show_blocking_modal) return ftxui::dbox({ main_layout | border, modal_box });

    return ftxui::dbox({
        main_layout | border | dim,
        modal_box
    });
}

bool Visualizer::handle_event(Event event, int mouse_dx) {
    if (show_file_modal_) {
        if (event == Event::Escape) { show_file_modal_ = false; return true; }
        int count = (int)file_list_.size();
        bool trigger_return = false;

        if (event.is_mouse()) {
            auto mouse = event.mouse();
            auto terminal = ftxui::Terminal::Size();
            int modal_w = 64;
            int items_h = 16; 
            int modal_h = is_save_mode_ ? 27 : 24; // title(1)+path(1)+sep(1)+list(18)+btn(1) + 1 outer top + 1 outer bottom = 24. (plus filename(3) for save)
            int start_x = (terminal.dimx - modal_w) / 2;
            int start_y = (terminal.dimy - modal_h) / 2;

            if (mouse.button == Mouse::WheelUp) { file_scroll_ = std::max(0, file_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
            if (mouse.button == Mouse::WheelDown) { file_scroll_ = std::min(std::max(0, count - 15), file_scroll_ + 1); screen_.PostEvent(Event::Custom); return true; }
            
            if (mouse.x >= start_x && mouse.x < start_x + modal_w) {
                int list_y = mouse.y - (start_y + 5);
                if (list_y >= 0 && list_y < items_h) {
                    file_selected_idx_ = list_y + file_scroll_ - 1;
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) trigger_return = true;
                    else { screen_.PostEvent(Event::Custom); return true; }
                }

                int btn_y = start_y + modal_h - 2;
                if (mouse.y == btn_y) {
                    int mid = start_x + modal_w / 2;
                    if (is_save_mode_ || is_dir_picker_) {
                        if (mouse.x < mid) file_selected_idx_ = count; // SAVE / SELECT
                        else file_selected_idx_ = count + 1; // CANCEL
                    } else {
                        // In Load mode, buttons start with CANCEL since SAVE is hidden
                        file_selected_idx_ = count + 1; 
                    }
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) trigger_return = true;
                    else { screen_.PostEvent(Event::Custom); return true; }
                }
            }
        }

        if (event == Event::ArrowUp) { file_selected_idx_ = std::max(-1, file_selected_idx_ - 1); return true; }
        if (event == Event::ArrowDown) { file_selected_idx_ = std::min(count + 1, file_selected_idx_ + 1); return true; }
        
        if (trigger_return || event == Event::Return || (!is_save_mode_ && !is_dir_picker_ && event == Event::Character(' '))) {
            if (file_selected_idx_ == count + 1) { // CANCEL
                show_file_modal_ = false;
                is_dir_picker_ = false;
            } else if ((is_save_mode_ || is_dir_picker_) && (event == Event::Return || file_selected_idx_ == count)) {
                // SAVE or DIRECTORY PICK
                if (is_save_mode_) {
                    if (!input_filename_.empty()) {
                        std::string full_path = (current_path_ / input_filename_).string();
                        if (full_path.find(".nathan") == std::string::npos) full_path += ".nathan";
                        save_config(full_path);
                        show_file_modal_ = false;
                    }
                } else if (is_dir_picker_) {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (auto& d : displays_) {
                        if (d->getName() == "Rosbag") {
#ifdef HAS_ROSBAG2
                            auto rosbag = std::dynamic_pointer_cast<RosbagDisplay>(d);
                            if (rosbag) {
                                if (rosbag->get_tab() == 0) rosbag->set_output_path(current_path_.string());
                                else rosbag->set_input_path(current_path_.string());
                            }
#endif
                            break;
                        }
                    }
                    show_file_modal_ = false;
                    is_dir_picker_ = false;
                }
            } else if (file_selected_idx_ == -1) { 
 // Up one level
                current_path_ = current_path_.parent_path();
                refresh_file_list();
                file_selected_idx_ = 0;
            } else if (file_selected_idx_ >= 0 && file_selected_idx_ < count) {
                if (file_list_[file_selected_idx_].is_directory()) {
                    current_path_ = file_list_[file_selected_idx_].path();
                    refresh_file_list();
                    file_selected_idx_ = 0;
                } else {
                    auto p = file_list_[file_selected_idx_].path();
                    if (is_save_mode_) {
                        input_filename_ = p.filename().string();
                    } else {
                        load_config(p.string());
                        show_file_modal_ = false;
                    }
                }
            }
            screen_.PostEvent(Event::Custom);
            return true;
        }

        if (is_save_mode_) {
            if (event == Event::Backspace) {
                if (!input_filename_.empty()) input_filename_.pop_back();
                screen_.PostEvent(Event::Custom);
                return true;
            }
            if (event.is_character()) {
                input_filename_ += event.character();
                screen_.PostEvent(Event::Custom);
                return true;
            }
        }
        return true;
    }
    if (show_frame_modal_) {
        if (event.is_mouse()) {
            auto mouse = event.mouse();
            if (mouse.button == Mouse::WheelUp) { modal_frame_scroll_ = std::max(0, modal_frame_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
            if (mouse.button == Mouse::WheelDown) { 
                int max_scroll = std::max(0, (int)available_frames_.size() - 15);
                modal_frame_scroll_ = std::min(max_scroll, modal_frame_scroll_ + 1); 
                screen_.PostEvent(Event::Custom); return true; 
            }
            
            auto terminal = ftxui::Terminal::Size();
            int modal_width = 44; 
            int frame_count = (int)available_frames_.size();
            int items_height = 15; // Matches max_visible in render_frame
            int modal_height = items_height + 7; 
            int start_x = (terminal.dimx - modal_width) / 2;
            int start_y = (terminal.dimy - modal_height) / 2;

            if (mouse.x >= start_x && mouse.x < start_x + modal_width &&
                mouse.y >= start_y && mouse.y < start_y + modal_height) {
                
                int relative_y = mouse.y - (start_y + 4); 
                if (relative_y >= 0 && relative_y < items_height) {
                    int hovered_idx = relative_y + modal_frame_scroll_;
                    if (hovered_idx < frame_count) {
                        modal_frame_selected_idx_ = hovered_idx;
                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            handle_event(Event::Return);
                        }
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }

                if (mouse.y == start_y + modal_height - 2) {
                    int middle = start_x + modal_width / 2;
                    if (mouse.x < middle) modal_frame_selected_idx_ = frame_count;
                    else modal_frame_selected_idx_ = frame_count + 1;
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        handle_event(Event::Return);
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
            }
            return true;
        }
        if (event == Event::ArrowUp) { modal_frame_selected_idx_ = std::max(0, modal_frame_selected_idx_ - 1); return true; }
        if (event == Event::ArrowDown) { modal_frame_selected_idx_ = std::min((int)available_frames_.size() + 1, modal_frame_selected_idx_ + 1); return true; }
        if (event == Event::Return) {
            if (modal_frame_selected_idx_ < (int)available_frames_.size()) {
                frame_idx_ = modal_frame_selected_idx_;
                if (frame_idx_ < (int)available_frames_.size()) fixed_frame_ = available_frames_[frame_idx_];
                show_frame_modal_ = false;
            } else if (modal_frame_selected_idx_ == (int)available_frames_.size()) { // OK
                show_frame_modal_ = false;
            } else if (modal_frame_selected_idx_ == (int)available_frames_.size() + 1) { // Cancel
                show_frame_modal_ = false;
            }
            screen_.PostEvent(Event::Custom);
            return true;
        }
        return true; 
    }

    if (show_plugin_modal_) {
        std::vector<int> filtered_indices;
        {
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (size_t i = 0; i < displays_.size(); ++i) {
                bool is_panel = (displays_[i]->getName() == "Nav2" || displays_[i]->getName() == "Teleop" || displays_[i]->getName() == "Rosbag" || displays_[i]->getName() == "MotionPlanning");
                if (modal_tab_idx_ == 0 && !is_panel) filtered_indices.push_back(i);
                else if (modal_tab_idx_ == 1 && is_panel) filtered_indices.push_back(i);
            }
        }
        int display_count = (int)filtered_indices.size();
        if (modal_tab_idx_ == 2) display_count = (int)modal_topic_entries_.size();

        if (event.is_mouse()) {
            auto mouse = event.mouse();
            if (mouse.button == Mouse::WheelUp) { modal_scroll_ = std::max(0, modal_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
            if (mouse.button == Mouse::WheelDown) { 
                int max_scroll = std::max(0, display_count - 15);
                modal_scroll_ = std::min(max_scroll, modal_scroll_ + 1); 
                screen_.PostEvent(Event::Custom); return true; 
            }

            auto terminal = ftxui::Terminal::Size();
            int modal_width = 50; 
            int max_h = 15; 
            int start_x = (terminal.dimx - modal_width) / 2;
            int start_y = (terminal.dimy - (max_h + 10)) / 2;

            if (mouse.x >= start_x && mouse.x < start_x + modal_width) {
                if (mouse.y == start_y + 3) {
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        int w = modal_width / 3;
                        if (mouse.x < start_x + w) { if (modal_tab_idx_ != 0) { modal_tab_idx_ = 0; modal_selected_idx_ = 0; modal_scroll_ = 0; } }
                        else if (mouse.x < start_x + 2*w) { if (modal_tab_idx_ != 1) { modal_tab_idx_ = 1; modal_selected_idx_ = 0; modal_scroll_ = 0; } }
                        else { if (modal_tab_idx_ != 2) { modal_tab_idx_ = 2; modal_selected_idx_ = 0; modal_scroll_ = 0; build_topic_tree(); } }
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }

                int relative_y = mouse.y - (start_y + 6); 
                if (relative_y >= 0 && relative_y < max_h) {
                    int hovered_idx = relative_y + modal_scroll_;
                    if (modal_tab_idx_ == 2) {
                        if (hovered_idx < (int)modal_topic_entries_.size()) {
                            modal_selected_idx_ = hovered_idx;
                            if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                                const auto& entry = modal_topic_entries_[modal_selected_idx_];
                                if (entry.is_plugin_type && entry.display_idx >= 0) {
                                    bool can_select = true;
                                    {
                                        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                                        if (displays_[entry.display_idx]->getName() == "Image") {
                                            int count = 0;
                                            for (auto const& [key, selected] : modal_topic_selections_) {
                                                if (selected && displays_[key.second]->getName() == "Image") count++;
                                            }
                                            if (count >= 2 && !modal_topic_selections_[{entry.topic, entry.display_idx}]) can_select = false;
                                        }
                                    }
                                    if (can_select) modal_topic_selections_[{entry.topic, entry.display_idx}] = !modal_topic_selections_[{entry.topic, entry.display_idx}];
                                } else if (!entry.is_plugin_type) {
                                    // Toggle expansion
                                    if (expanded_topic_nodes_.count(entry.topic)) {
                                        expanded_topic_nodes_.erase(entry.topic);
                                    } else {
                                        expanded_topic_nodes_.insert(entry.topic);
                                    }
                                    build_topic_tree();
                                }
                            }
                        }
                    } else if (hovered_idx < display_count) {
                        modal_selected_idx_ = hovered_idx; 
                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                            int original_idx = filtered_indices[modal_selected_idx_];
                            modal_plugin_states_[original_idx] = !modal_plugin_states_[original_idx];
                        }
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }

                if (mouse.y == start_y + max_h + 7) {
                    int middle = start_x + modal_width / 2;
                    int final_count = (modal_tab_idx_ == 2) ? (int)modal_topic_entries_.size() : display_count;
                    if (mouse.x < middle) modal_selected_idx_ = final_count;
                    else modal_selected_idx_ = final_count + 1;

                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        handle_event(Event::Return);
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
            }
            return true;
        }
        if (event == Event::ArrowUp) { modal_selected_idx_ = std::max(0, modal_selected_idx_ - 1); return true; }
        if (event == Event::ArrowDown) { modal_selected_idx_ = std::min(display_count + 1, modal_selected_idx_ + 1); return true; }
        if (event == Event::ArrowLeft) { if (modal_tab_idx_ != 0) { modal_tab_idx_ = 0; modal_selected_idx_ = 0; modal_scroll_ = 0; } return true; }
        if (event == Event::ArrowRight) { if (modal_tab_idx_ != 1) { modal_tab_idx_ = 1; modal_selected_idx_ = 0; modal_scroll_ = 0; } else if (modal_tab_idx_ == 1) { modal_tab_idx_ = 2; modal_selected_idx_ = 0; modal_scroll_ = 0; build_topic_tree(); } return true; }
        
        if (event == Event::Character(' ')) {
            if (modal_tab_idx_ == 2) {
                if (modal_selected_idx_ < (int)modal_topic_entries_.size()) {
                    const auto& entry = modal_topic_entries_[modal_selected_idx_];
                    if (entry.is_plugin_type && entry.display_idx >= 0) {
                        bool can_select = true;
                        {
                            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                            if (displays_[entry.display_idx]->getName() == "Image") {
                                int count = 0;
                                for (auto const& [key, selected] : modal_topic_selections_) {
                                    if (selected && displays_[key.second]->getName() == "Image") count++;
                                }
                                if (count >= 2 && !modal_topic_selections_[{entry.topic, entry.display_idx}]) can_select = false;
                            }
                        }
                        if (can_select) modal_topic_selections_[{entry.topic, entry.display_idx}] = !modal_topic_selections_[{entry.topic, entry.display_idx}];
                    } else if (!entry.is_plugin_type) {
                        // Toggle expansion
                        if (expanded_topic_nodes_.count(entry.topic)) {
                            expanded_topic_nodes_.erase(entry.topic);
                        } else {
                            expanded_topic_nodes_.insert(entry.topic);
                        }
                        build_topic_tree();
                    }
                }
            } else if (modal_selected_idx_ < display_count) {
                std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                int original_idx = filtered_indices[modal_selected_idx_];
                modal_plugin_states_[original_idx] = !modal_plugin_states_[original_idx];
            }
            return true;
        }
        if (event == Event::Return) {
            if (modal_selected_idx_ == display_count && modal_tab_idx_ < 2) { // OK Plugins/Panels
                {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (size_t i = 0; i < displays_.size(); ++i) {
                        if (i < 64) {
                            bool was_added = displays_[i]->isAdded();
                            bool now_added = modal_plugin_states_[i];
                            displays_[i]->setAdded(now_added);
                            if (!was_added && now_added) displays_[i]->setEnabled(true);
                        }
                    }
                }
                show_plugin_modal_ = false;
                if (plugin_idx_ < 0 || (plugin_idx_ < (int)displays_.size() && !displays_[plugin_idx_]->isAdded())) {
                    plugin_idx_ = -1;
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (size_t i = 0; i < displays_.size(); ++i) if (displays_[i]->isAdded()) { plugin_idx_ = (int)i; break; }
                }
                discover_topics();
            } else if (modal_tab_idx_ == 2 && modal_selected_idx_ == (int)modal_topic_entries_.size()) { // OK Topics
                {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (auto const& [key, selected] : modal_topic_selections_) {
                        const std::string& topic = key.first;
                        int p_idx = key.second;
                        if (p_idx >= 0 && p_idx < (int)displays_.size()) {
                            bool currently_enabled = displays_[p_idx]->isTopicEnabled(topic);
                            if (selected != currently_enabled) {
                                displays_[p_idx]->setTopic(topic);
                            }
                            if (selected) {
                                displays_[p_idx]->setAdded(true);
                                displays_[p_idx]->setEnabled(true);
                            }
                        }
                    }
                }
                show_plugin_modal_ = false;
                if (plugin_idx_ < 0 || (plugin_idx_ < (int)displays_.size() && !displays_[plugin_idx_]->isAdded())) {
                    plugin_idx_ = -1;
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    for (size_t i = 0; i < displays_.size(); ++i) if (displays_[i]->isAdded()) { plugin_idx_ = (int)i; break; }
                }
                discover_topics();
            } else if (modal_selected_idx_ == (modal_tab_idx_ == 2 ? (int)modal_topic_entries_.size() : display_count) + 1) { // Cancel
                show_plugin_modal_ = false;
            } else if (modal_tab_idx_ == 2) {
                if (modal_selected_idx_ < (int)modal_topic_entries_.size()) {
                    const auto& entry = modal_topic_entries_[modal_selected_idx_];
                    if (entry.is_plugin_type && entry.display_idx >= 0) {
                        bool can_select = true;
                        {
                            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                            if (displays_[entry.display_idx]->getName() == "Image") {
                                int count = 0;
                                for (auto const& [key, selected] : modal_topic_selections_) {
                                    if (selected && displays_[key.second]->getName() == "Image") count++;
                                }
                                if (count >= 2 && !modal_topic_selections_[{entry.topic, entry.display_idx}]) can_select = false;
                            }
                        }
                        if (can_select) modal_topic_selections_[{entry.topic, entry.display_idx}] = !modal_topic_selections_[{entry.topic, entry.display_idx}];
                    } else if (!entry.is_plugin_type) {
                        // Toggle expansion
                        if (expanded_topic_nodes_.count(entry.topic)) {
                            expanded_topic_nodes_.erase(entry.topic);
                        } else {
                            expanded_topic_nodes_.insert(entry.topic);
                        }
                        build_topic_tree();
                    }
                }
            } else if (modal_selected_idx_ < display_count) {
                std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                int original_idx = filtered_indices[modal_selected_idx_];
                modal_plugin_states_[original_idx] = !modal_plugin_states_[original_idx];
            }
            screen_.PostEvent(Event::Custom);
            return true;
        }
        return true; 
    }

    if (show_topic_modal_) {
        if (event.is_mouse()) {
            auto mouse = event.mouse();
            if (mouse.button == Mouse::WheelUp) { topic_modal_scroll_ = std::max(0, topic_modal_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
            if (mouse.button == Mouse::WheelDown) { 
                int max_scroll = std::max(0, (int)topic_modal_list_.size() - 15);
                topic_modal_scroll_ = std::min(max_scroll, topic_modal_scroll_ + 1); 
                screen_.PostEvent(Event::Custom); return true; 
            }

            auto terminal = ftxui::Terminal::Size();
            int modal_width = 60; 
            int topic_count = (int)topic_modal_list_.size();
            int items_height = std::min(15, topic_count);
            int modal_height = items_height + 7; 
            int start_x = std::clamp(topic_modal_x_ - modal_width / 2, 0, terminal.dimx - modal_width);
            int start_y = std::clamp(topic_modal_y_ - modal_height / 2, 0, terminal.dimy - modal_height);

            if (mouse.x >= start_x && mouse.x < start_x + modal_width &&
                mouse.y >= start_y && mouse.y < start_y + modal_height) {
                
                int relative_y = mouse.y - (start_y + 4); 
                if (relative_y >= 0 && relative_y < items_height) {
                    int hovered_idx = relative_y + topic_modal_scroll_;
                    if (hovered_idx < topic_count) {
                        topic_modal_selected_idx_ = hovered_idx;
                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            handle_event(Event::Return);
                        }
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }

                if (mouse.y == start_y + modal_height - 2) {
                    topic_modal_selected_idx_ = topic_count;
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        handle_event(Event::Return);
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
            }
            return true;
        }
        if (event == Event::ArrowUp) { topic_modal_selected_idx_ = std::max(0, topic_modal_selected_idx_ - 1); return true; }
        if (event == Event::ArrowDown) { topic_modal_selected_idx_ = std::min((int)topic_modal_list_.size(), topic_modal_selected_idx_ + 1); return true; }
        if (event == Event::Return) {
            if (topic_modal_selected_idx_ < (int)topic_modal_list_.size()) {
                std::string selected_topic = topic_modal_list_[topic_modal_selected_idx_];
                std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                if (topic_target_display_) {
                    if (topic_target_display_->getMessageType() == "sensor_msgs/msg/Image") {
                        auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(topic_target_display_);
                        if (img_disp) img_disp->replaceTopic(topic_target_slot_, selected_topic);
                    } else {
                        topic_target_display_->setTopic(selected_topic);
                    }
                }
                show_topic_modal_ = false;
            } else { // Cancel
                show_topic_modal_ = false;
            }
            screen_.PostEvent(Event::Custom);
            return true;
        }
        return true;
    }

    if (show_config_modal_) {
        bool was_dragging = is_dragging_config_;
        if (event.is_mouse()) {
            auto mouse = event.mouse();
            if (mouse.motion == Mouse::Pressed) is_dragging_config_ = true;
            if (mouse.motion == Mouse::Released) is_dragging_config_ = false;
        }

        TopicConfig cfg;
        {
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            cfg = config_target_display_->getTopicConfig(config_modal_topic_);
        }
        if (ConfigHelper::handle_edit_event(event, cfg, config_modal_selected_idx_,
                                          show_config_modal_, right_width_, 
                                          config_target_display_->getName(),
                                          mouse_dx,
                                          was_dragging)) {

            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            config_target_display_->setTopicConfig(config_modal_topic_, cfg);
            screen_.PostEvent(Event::Custom);
            if (!show_config_modal_) is_dragging_config_ = false; 
            return true;
        }
        return true;
    }

    if (event.is_mouse()) {
        auto mouse = event.mouse();
        auto terminal = ftxui::Terminal::Size();

        if (mouse.motion == Mouse::Released) {
            is_dragging_config_ = false;
            if (is_dragging_seek_) {
                is_dragging_seek_ = false;
                std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                for (auto& d : displays_) {
                    if (d->getName() == "Rosbag") {
#ifdef HAS_ROSBAG2
                        auto rb = std::dynamic_pointer_cast<RosbagDisplay>(d);
                        if (rb) rb->finish_scrubbing();
#endif
                        break;
                    }
                }
            }
        }

        if (is_dragging_seek_) {
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
#ifdef HAS_ROSBAG2
            std::shared_ptr<RosbagDisplay> rosbag = nullptr;
            for (auto& d : displays_) if (d->getName() == "Rosbag") { rosbag = std::dynamic_pointer_cast<RosbagDisplay>(d); break; }
            if (rosbag) {
                int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                float progress = std::clamp(static_cast<float>(rx - 7) / (right_width_ - 15), 0.0f, 1.0f);
                rosbag->seek(progress);
            }
#endif
            screen_.PostEvent(Event::Custom);
            return true;
        }

        if (mouse.y >= 0 && mouse.y <= 2) { // Top bar area
            if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                // Approximate positions
                int x = mouse.x;
                int save_start = 15 + 1 + (int)fixed_frame_.size() + 8 + 1; // " TViz " + sep + " Frame: map " + sep
                if (x >= save_start && x < save_start + 8) {
                    show_file_modal_ = true;
                    is_save_mode_ = true;
                    input_filename_ = "config.nathan";
                    refresh_file_list();
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
                if (x >= save_start + 8 && x < save_start + 16) {
                    show_file_modal_ = true;
                    is_save_mode_ = false;
                    refresh_file_list();
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
            }
        }

        if (mouse.x < 31) {
            int y_cursor = 3; 

            if (mouse.y >= y_cursor && mouse.y < y_cursor + 6) {
                if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                    show_frame_modal_ = true;
                    modal_frame_selected_idx_ = frame_idx_;
                    screen_.PostEvent(Event::Custom);
                }
                return true;
            }
            y_cursor += 6;

            if (mouse.y >= y_cursor && mouse.y < y_cursor + 6) {
                if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                    cycle_tool();
                    screen_.PostEvent(Event::Custom);
                }
                return true;
            }
            y_cursor += 6;

            int plugin_box_height = 13;
            if (mouse.y >= y_cursor && mouse.y < y_cursor + plugin_box_height) {
                if (mouse.button == Mouse::WheelUp)   { plugin_scroll_ = std::max(0, plugin_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                if (mouse.button == Mouse::WheelDown) { plugin_scroll_++; screen_.PostEvent(Event::Custom); return true; }
                
                if (mouse.y == y_cursor + 1) {
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        show_plugin_modal_ = true;
                        modal_selected_idx_ = 0;
                        modal_tab_idx_ = 0; 
                        modal_scroll_ = 0;
                        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                        for (size_t i = 0; i < displays_.size(); ++i) if (i < 64) modal_plugin_states_[i] = displays_[i]->isAdded();
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
                int list_y = mouse.y - (y_cursor + 3) + plugin_scroll_;
                if (list_y >= 0) {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    int count = 0;
                    bool found = false;
                    for (size_t i = 0; i < displays_.size(); ++i) {
                        if (displays_[i]->isAdded()) {
                            if (count == list_y) {
                                sidebar_hover_idx_ = i;
                                if (mouse.motion == Mouse::Pressed) {
                                    if (mouse.button == Mouse::Left) {
                                        plugin_idx_ = i;
                                        discover_topics();
                                    } else if (mouse.button == Mouse::Right) {
                                        displays_[i]->setEnabled(!displays_[i]->isEnabled());
                                    }
                                }
                                found = true;
                                break;
                            }
                            count++;
                        }
                    }
                    if (!found) sidebar_hover_idx_ = -1;
                } else {
                    sidebar_hover_idx_ = -1;
                }
                screen_.PostEvent(Event::Custom);
                return true;
            }
            y_cursor += plugin_box_height;

            if (mouse.y >= y_cursor && mouse.y < ftxui::Terminal::Size().dimy - 4) {
                if (mouse.button == Mouse::WheelUp)   { topic_scroll_ = std::max(0, topic_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                if (mouse.button == Mouse::WheelDown) { topic_scroll_++; screen_.PostEvent(Event::Custom); return true; }
                
                int list_y = mouse.y - (y_cursor + 3) + topic_scroll_;
                if (list_y >= 0 && list_y < (int)available_topics_.size()) {
                    topic_idx_ = list_y;
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                    if (plugin_idx_ >= 0) {
                        auto disp = displays_[plugin_idx_];
                        std::string type = disp->getMessageType();
                        std::string target_topic = available_topics_[topic_idx_];

                        if (mouse.motion == Mouse::Pressed) {
                            if (type == "TF") {
                                auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(disp);
                                if (tf_disp) {
                                    if (mouse.button == Mouse::Left || mouse.button == Mouse::Right)
                                        tf_disp->toggleFrame(target_topic);
                                }
                            } else if (type == "sensor_msgs/msg/Image") {
                                auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(disp);
                                if (img_disp) {
                                    bool enabled = img_disp->isTopicEnabled(target_topic);
                                    if (mouse.button == Mouse::Left && !enabled) {
                                        img_disp->setTopic(target_topic);
                                    } else if (mouse.button == Mouse::Right && enabled) {
                                        img_disp->setTopic(target_topic);
                                    }
                                }
                            } else if (type != "None") {
                                if (mouse.button == Mouse::Left || mouse.button == Mouse::Right) {
                                    disp->setTopic(target_topic);
                                }
                            }
                        }
                    }
                    screen_.PostEvent(Event::Custom);
                }
                return true;
            }
        } else if (right_width_ > 0 && mouse.x >= terminal.dimx - right_width_ - 1) {
            int ry = mouse.y - 3; 
            if (ry >= 0) {
                int bottom_h = 14;
                bool in_bottom = (mouse.y >= terminal.dimy - bottom_h - 1);

                if (!in_bottom) {
                    std::shared_ptr<ImageDisplay> img_disp = nullptr;
                    bool nav2_active = false;
                    {
                        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
                        for (auto& d : displays_) {
                            if (d->isAdded() && d->isEnabled()) {
                                if (d->getName() == "Image") img_disp = std::dynamic_pointer_cast<ImageDisplay>(d);
                                if (d->getName() == "Nav2") nav2_active = true;
                            }
                        }
                    }

                    if (img_disp) {
                        int n = img_disp->getEnabledTopicCount();
                        if (n > 0) {
                            int available_char_h = std::max(10, terminal.dimy - 4); 
                            if (nav2_active || (plugin_idx_ >= 0 && displays_[plugin_idx_]->getName() != "Image" && displays_[plugin_idx_]->getName() != "Nav2" && displays_[plugin_idx_]->isEnabled())) available_char_h -= bottom_h;
                            int h = available_char_h / n;

                            int slot = ry / h;
                            if (slot < n) {
                                int slot_ry = ry % h;
                                if (slot_ry <= 2) { 
                                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                                        topic_target_display_ = img_disp;
                                        topic_target_slot_ = slot;
                                        topic_modal_selected_idx_ = 0;
                                        topic_modal_x_ = terminal.dimx - right_width_ / 2 - 1;
                                        topic_modal_y_ = 3 + slot * h + h / 2;
                                        
                                        topic_modal_list_.clear();
                                        auto topic_map = node_->get_topic_names_and_types();
                                        for (const auto& [name, types] : topic_map) {
                                            for (const auto& type : types) {
                                                if (type == "sensor_msgs/msg/Image" || type == "sensor_msgs/Image") {
                                                    topic_modal_list_.push_back(name);
                                                    break;
                                                }
                                            }
                                        }
                                        std::sort(topic_modal_list_.begin(), topic_modal_list_.end());
                                        show_topic_modal_ = true;
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    } else if (mouse.button == Mouse::Right && mouse.motion == Mouse::Pressed) {
                                        auto enabled_topics = img_disp->getEnabledTopics();
                                        if (slot < (int)enabled_topics.size()) {
                                            img_disp->setTopic(enabled_topics[slot]);
                                            screen_.PostEvent(Event::Custom);
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {
                    std::lock_guard<std::recursive_mutex> lock(displays_mutex_);

                    // Priority should match render_frame: config_panel > rosbag_panel > teleop_panel > nav2_panel
                    bool config_active = false;
                    std::shared_ptr<Display> active_disp = nullptr;
                    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
                        active_disp = displays_[plugin_idx_];
                        if (active_disp->getName() != "Image" && active_disp->getName() != "Nav2" && 
                            active_disp->getName() != "Teleop" && active_disp->getName() != "Rosbag" && 
                            active_disp->isAdded() && active_disp->isEnabled()) {
                            config_active = true;
                        }
                    }

#ifdef HAS_ROSBAG2
                    std::shared_ptr<RosbagDisplay> rosbag = nullptr;
                    for (auto& d : displays_) if (d->getName() == "Rosbag" && d->isAdded() && d->isEnabled()) { rosbag = std::dynamic_pointer_cast<RosbagDisplay>(d); break; }
#endif

                    if (config_active) {
                        if (mouse.button == Mouse::WheelUp)   { config_scroll_ = std::max(0, config_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                        if (mouse.button == Mouse::WheelDown) { config_scroll_++; screen_.PostEvent(Event::Custom); return true; }

                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            int cry = mouse.y - (terminal.dimy - 12); 
                            if (cry >= 0 && cry < 10) {
                                int item_idx = cry + config_scroll_;
                                auto topics = active_disp->getEnabledTopics();
                                if (item_idx >= 0 && item_idx < (int)topics.size()) {
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx > right_width_ - 10 || active_disp->getName() != "Image") {
                                        // CLICKED [EDIT] or TOPIC NAME (for non-image) -> Open Topic SETTINGS
                                        show_config_modal_ = true;
                                        config_modal_topic_ = topics[item_idx];
                                        config_target_display_ = active_disp;
                                        config_modal_selected_idx_ = 0;
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    } else {
                                        // CLICKED TOPIC NAME (for Image) -> Open Topic SELECTION (to change topic)
                                        topic_target_display_ = active_disp;
                                        topic_target_slot_ = item_idx;
                                        topic_modal_selected_idx_ = 0;
                                        topic_modal_x_ = mouse.x - 20;
                                        topic_modal_y_ = mouse.y;

                                        topic_modal_list_.clear();
                                        std::string target_type = active_disp->getMessageType();
                                        auto topic_map = node_->get_topic_names_and_types();
                                        for (const auto& [name, types] : topic_map) {
                                            for (const auto& type : types) {
                                                if (type == target_type) {
                                                    topic_modal_list_.push_back(name);
                                                    break;
                                                }
                                            }
                                        }
                                        std::sort(topic_modal_list_.begin(), topic_modal_list_.end());
                                        show_topic_modal_ = true;
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    }
                                }

                            }
                        }
                    } 
#ifdef HAS_ROSBAG2
                    else if (rosbag) {
                        int cry = mouse.y - (terminal.dimy - 14);
                        if (mouse.button == Mouse::WheelUp)   { config_scroll_ = std::max(0, config_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                        if (mouse.button == Mouse::WheelDown) { config_scroll_++; screen_.PostEvent(Event::Custom); return true; }

                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            // Top bar of panel (Tabs)
                            if (cry == 0) {
                                int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                if (rx > right_width_ - 16) {
                                    if (rx < right_width_ - 8) rosbag->set_tab(0);
                                    else rosbag->set_tab(1);
                                    screen_.PostEvent(Event::Custom);
                                    return true;
                                }
                            }

                            // Check for [EDIT] button in other display settings (This was redundant/conflicting logic, now handled by config_active)

                            if (rosbag->get_tab() == 0) { // RECORD
                                if (cry == 2) { // Output Path change
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx > right_width_ - 12) { // Only if clicked on [CHANGE]
                                        show_file_modal_ = true;
                                        is_dir_picker_ = true;
                                        refresh_file_list();
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    }
                                } else if (cry >= 4 && cry <= 7) { // Topic list (scrolled)
                                    auto topic_names = node_->get_topic_names_and_types();
                                    std::vector<std::string> sorted_topics;
                                    for (auto const& [name, types] : topic_names) sorted_topics.push_back(name);
                                    std::sort(sorted_topics.begin(), sorted_topics.end());
                                    int idx = cry - 4 + config_scroll_;
                                    if (idx >= 0 && idx < (int)sorted_topics.size()) {
                                        rosbag->toggle_topic(sorted_topics[idx]);
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    }
                                } else if (cry == 9 || cry == 10 || cry == 11) { // Buttons row
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx < right_width_ / 2) rosbag->start_recording();
                                    else rosbag->stop_recording();
                                    screen_.PostEvent(Event::Custom);
                                    return true;
                                }
                            } else { // PLAY
                                if (cry == 2) { // Input Path selection
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx > right_width_ - 12) { // Only if clicked on [SELECT]
                                        show_file_modal_ = true;
                                        is_dir_picker_ = true; // Still use dir picker since bags are folders
                                        refresh_file_list();
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    }
                                } else if (cry == 4) { // Rate, Loop, and Pause row
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx >= 7 && rx <= 11) rosbag->set_playback_rate(rosbag->get_playback_rate() - 0.1f);
                                    else if (rx >= 16 && rx <= 20) rosbag->set_playback_rate(rosbag->get_playback_rate() + 0.1f);
                                    else if (rx >= 25 && rx <= 35) rosbag->toggle_loop();
                                    else if (rx > right_width_ - 12) rosbag->toggle_pause();
                                    screen_.PostEvent(Event::Custom);
                                    return true;
                                } else if (cry == 6) { // Seek row
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (mouse.motion == Mouse::Pressed) {
                                        is_dragging_seek_ = true;
                                        float progress = std::clamp(static_cast<float>(rx - 7) / (right_width_ - 15), 0.0f, 1.0f);
                                        rosbag->start_scrubbing(progress);
                                        rosbag->seek(progress);
                                    }
                                    screen_.PostEvent(Event::Custom);
                                    return true;
                                } else if (cry >= 8 && cry <= 10) { // Play buttons row
                                    int rx = mouse.x - (terminal.dimx - right_width_ - 1);
                                    if (rx < right_width_ / 2) rosbag->start_playback();
                                    else rosbag->stop_playback();
                                    screen_.PostEvent(Event::Custom);
                                    return true;
                                }
                            }
                        }
                    }
#endif

                    // Fallback to Teleop/Nav2 or custom display event handling
                    if (active_disp) {
                        if (active_disp->handle_event(event, config_scroll_)) return true;
                    }
                }
            }
            return true;
        }
    }

    // --- PRIORITY: Active Plugin Handlers (Input capture) ---
    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Nav2" && d->isAdded() && d->isEnabled()) {
                if (event == Event::Return ||
                    event == Event::Character('c') || event == Event::Character('C') ||
                    event == Event::Special("\x7F") || event == Event::Backspace) {
                    if (d->handle_event(event)) return true;
                }
            }
            if (d->getName() == "MotionPlanning" && d->isAdded() && d->isEnabled()) {
                if (d->handle_event(event)) return true;
            }
        }
    }

    if (event == Event::Character('p') || event == Event::Character('P')) {
        show_plugin_modal_ = !show_plugin_modal_;
        if (show_plugin_modal_) {
            modal_selected_idx_ = 0;
            modal_tab_idx_ = 0; 
            modal_scroll_ = 0;
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (size_t i = 0; i < displays_.size(); ++i) {
                if (i < 64) modal_plugin_states_[i] = displays_[i]->isAdded();
            }
            // Initialize topic selections
            modal_topic_selections_.clear();
            for (size_t i = 0; i < displays_.size(); ++i) {
                if (displays_[i]->isAdded()) {
                    auto topics = displays_[i]->getEnabledTopics();
                    for (const auto& t : topics) {
                        modal_topic_selections_[{t, (int)i}] = true;
                    }
                }
            }
        }
        screen_.PostEvent(Event::Custom);
        return true;
    }

    if (event == Event::Character('f') || event == Event::Character('F')) {
        show_frame_modal_ = !show_frame_modal_;
        if (show_frame_modal_) {
            modal_frame_selected_idx_ = frame_idx_;
        }
        screen_.PostEvent(Event::Custom);
        return true;
    }

    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        for (auto& d : displays_) {
            if (d->getName() == "Teleop" && d->isAdded() && d->isEnabled()) {
                if (d->handle_event(event)) return true;
            }
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            if (displays_[plugin_idx_]->handle_event(event)) return true;
        }
    }

    if (event == Event::ArrowUp)    { tar_cam_z_ = tar_cam_z_.load() + 0.5f; return true; }
    if (event == Event::ArrowDown)  { tar_cam_z_ = tar_cam_z_.load() - 0.5f; return true; }
    if (event == Event::ArrowLeft)  { tar_cam_y_ = tar_cam_y_.load() + 0.5f; return true; }
    if (event == Event::ArrowRight) { tar_cam_y_ = tar_cam_y_.load() - 0.5f; return true; }
    if (event == Event::PageUp)     { tar_cam_x_ = tar_cam_x_.load() + 0.5f; return true; }
    if (event == Event::PageDown)   { tar_cam_x_ = tar_cam_x_.load() - 0.5f; return true; }
    if (event == Event::Character('a')) { tar_yaw_ = tar_yaw_.load() - 0.1f; return true; }
    if (event == Event::Character('d')) { tar_yaw_ = tar_yaw_.load() + 0.1f; return true; }
    if (event == Event::Character('w')) { tar_pitch_ = tar_pitch_.load() + 0.1f; return true; }
    if (event == Event::Character('s')) { tar_pitch_ = tar_pitch_.load() - 0.1f; return true; }
    if (event == Event::Character('r') || event == Event::Character('R')) {
        tar_yaw_ = 0.0f; tar_pitch_ = 0.5f; tar_dist_ = 5.0f;
        tar_cam_x_ = 0.0f; tar_cam_y_ = 0.0f; tar_cam_z_ = 0.0f; tar_zoom_ = 250.0f;
        return true;
    }
    if (event == Event::Character('t') || event == Event::Character('T')) {
        float cx = 0.0f, cy = 0.0f, cz = 0.0f, zoom = 100.0f, yaw = 0.0f;
        bool found_map = false;
        {
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            for (auto& d : displays_) {
                if (d->getName() == "Map") {
                    auto md = std::dynamic_pointer_cast<MapDisplay>(d);
                    if (md) {
                        float raw_cx = md->getCenterX(), raw_cy = md->getCenterY();
                        float mw = md->getWidth(), mh = md->getHeight();
                        std::string map_frame = md->getFrameId();
                        
                        try {
                            auto t = tf_buffer_->lookupTransform(fixed_frame_, map_frame, tf2::TimePointZero);
                            tf2::Vector3 map_origin(raw_cx, raw_cy, 0.0f);
                            tf2::Transform tf; tf2::fromMsg(t.transform, tf);
                            tf2::Vector3 target = tf * map_origin;
                            cx = (float)target.x(); cy = (float)target.y(); cz = (float)target.z();
                            
                            // Align camera yaw with map yaw
                            double r, p, y;
                            tf.getBasis().getRPY(r, p, y);
                            yaw = (float)y;
                            found_map = true;
                        } catch (...) {
                            cx = raw_cx; cy = raw_cy; 
                        }
                        
                        float max_dim = std::max(mw, mh);
                        if (max_dim > 0.1f) zoom = 1500.0f / max_dim; 
                    }
                    break;
                }
            }
        }
        tar_pitch_ = 1.57f; // Top-down
        tar_yaw_ = yaw;
        tar_dist_ = 10.0f;
        tar_cam_x_ = cx; tar_cam_y_ = cy; tar_cam_z_ = cz; 
        tar_zoom_ = zoom;
        return true;
    }
    if (event == Event::Character('+') || event == Event::Character('=')) { tar_zoom_ = tar_zoom_.load() * 1.1f; return true; }
    if (event == Event::Character('-') || event == Event::Character('_')) { tar_zoom_ = tar_zoom_.load() / 1.1f; return true; }
    if (event == Event::Character('g') || event == Event::Character('G')) { if (grid_display_) grid_display_->toggle(); return true; }
    if (event == Event::Character('v') || event == Event::Character('V')) { cycle_tool(); return true; }
    if (event == Event::Tab) {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        int count = 0;
        do {
            plugin_idx_ = (plugin_idx_ + 1) % (int)displays_.size();
            count++;
        } while (!displays_[plugin_idx_]->isAdded() && count < (int)displays_.size());
        
        if (!displays_[plugin_idx_]->isAdded()) plugin_idx_ = -1;
        discover_topics();
        return true;
    }
    if (event == Event::Character('f') || event == Event::Character('F')) {
        if (!available_frames_.empty()) { frame_idx_ = (frame_idx_ + 1) % available_frames_.size(); fixed_frame_ = available_frames_[frame_idx_]; }
        return true;
    }
    if (event == Event::Character('y') || event == Event::Character('Y')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ - 1 + available_topics_.size()) % available_topics_.size();
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            if (plugin_idx_ >= 0) {
                auto disp = displays_[plugin_idx_]; std::string type = disp->getMessageType();
                if (type != "TF" && type != "sensor_msgs/msg/Image" && type != "None") disp->setTopic(available_topics_[topic_idx_]);
            }
        }
        return true;
    }
    if (event == Event::Character('h') || event == Event::Character('H')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ + 1) % available_topics_.size();
            std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
            if (plugin_idx_ >= 0) {
                auto disp = displays_[plugin_idx_]; std::string type = disp->getMessageType();
                if (type != "TF" && type != "sensor_msgs/msg/Image" && type != "None") disp->setTopic(available_topics_[topic_idx_]);
            }
        }
        return true;
    }
    if (event == Event::Character(' ')) {
        std::lock_guard<std::recursive_mutex> lock(displays_mutex_);
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            auto disp = displays_[plugin_idx_];
            if (disp->getMessageType() == "TF" && !available_topics_.empty()) {
                auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(disp);
                if (tf_disp) { tf_disp->toggleFrame(available_topics_[topic_idx_]); return true; }
            } else if (!available_topics_.empty()) { disp->setTopic(available_topics_[topic_idx_]); return true; }
        }
    }
    if (event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }
    return false;
}

} // namespace terminal_rviz
