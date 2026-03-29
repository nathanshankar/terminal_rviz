#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"
#include "terminal_rviz/displays/nav2_display.hpp"
#include "terminal_rviz/displays/map_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    : node_(node), screen_(ScreenInteractive::TerminalOutput()) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

void Visualizer::add_display(std::shared_ptr<Display> display) {
    displays_.push_back(display);
}

void Visualizer::cycle_tool() {
    if (current_tool_ == Tool::Nav2) current_tool_ = Tool::Orbit;
    else if (current_tool_ == Tool::Orbit) current_tool_ = Tool::Pan;
    else current_tool_ = Tool::Nav2;
    last_mouse_x_ = 0; last_mouse_y_ = 0;
}

std::string Visualizer::get_tool_name() const {
    if (current_tool_ == Tool::Nav2) return "NAV2";
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

            std::stringstream ss;
            ss << "T:" << get_tool_name() << " B:" << (int)mouse.button << " M:" << (int)mouse.motion;
            status_msg_ = ss.str();

            if (mouse.button == Mouse::WheelUp)   { tar_zoom_ = tar_zoom_.load() * 1.1f; return true; }
            if (mouse.button == Mouse::WheelDown) { tar_zoom_ = tar_zoom_.load() / 1.1f; return true; }

            bool is_pressed = (mouse.motion == Mouse::Pressed);
            bool is_released = (mouse.motion == Mouse::Released);

            if (mouse.button == Mouse::Left) {
                if (current_tool_ == Tool::Orbit) {
                    if (is_pressed && std::abs(dx) < 50) { tar_yaw_ = tar_yaw_.load() + dx * 0.015f; tar_pitch_ = tar_pitch_.load() - dy * 0.015f; }
                } else if (current_tool_ == Tool::Pan) {
                    if (is_pressed && std::abs(dx) < 50) {
                        float f = tar_dist_.load() * 0.005f, y = cur_yaw_;
                        float dy_boosted = dy * 2.5f; 
                        tar_cam_x_ = tar_cam_x_.load() - (dy_boosted * std::cos(y) + dx * std::sin(y)) * f;
                        tar_cam_y_ = tar_cam_y_.load() - (dy_boosted * std::sin(y) - dx * std::cos(y)) * f;
                    }
                } else if (current_tool_ == Tool::Nav2) {
                    std::shared_ptr<Nav2Display> nav2 = nullptr;
                    for (auto& d : displays_) if (d->getName() == "Nav2") { nav2 = std::dynamic_pointer_cast<Nav2Display>(d); break; }
                    if (nav2 && nav2->isEnabled()) {
                        int vx = mouse.x - canvas_x_offset_;
                        int vy = mouse.y - canvas_y_offset_;
                        if (vx >= 0 && vy >= 0) {
                            float wx, wy; bool hit = renderer_.pick_ground_plane(vx * 2, vy * 4, wx, wy);
                            if (is_pressed && hit) {
                                if (!nav2->is_selecting()) { nav2->set_goal(wx, wy, fixed_frame_); nav2->start_selection(); }
                                else nav2->update_goal_orientation(wx, wy);
                            } else if (is_released) {
                                if (hit) nav2->update_goal_orientation(wx, wy);
                                nav2->finalize_selection();
                            }
                        }
                    }
                }
                return true;
            }
            if (mouse.button == Mouse::Right && is_pressed) {
                tar_yaw_ = tar_yaw_.load() + dx * 0.015f; tar_pitch_ = tar_pitch_.load() - dy * 0.015f; return true;
            }
            if (mouse.button == Mouse::Middle && is_pressed) {
                float f = tar_dist_.load() * 0.005f, y = cur_yaw_;
                tar_cam_x_ = tar_cam_x_.load() - (dy * 2.5f * std::cos(y) + dx * std::sin(y)) * f;
                tar_cam_y_ = tar_cam_y_.load() - (dy * 2.5f * std::sin(y) - dx * std::cos(y)) * f;
                return true;
            }
            return true;
        }
        return handle_event(event); 
    });

    std::thread ui_thread([&]() {
        while (!quit_flag_ && rclcpp::ok()) { discover_frames(); discover_topics(); screen_.PostEvent(Event::Custom); std::this_thread::sleep_for(50ms); }
        screen_.Exit();
    });

    screen_.Loop(component);
    quit_flag_ = true; if (ui_thread.joinable()) ui_thread.join();
}

void Visualizer::stop() { quit_flag_ = true; screen_.Exit(); }

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

void Visualizer::discover_topics() {
    if (plugin_idx_ < 0 || static_cast<size_t>(plugin_idx_) >= displays_.size()) { available_topics_.clear(); return; }
    auto display = displays_[plugin_idx_];
    std::string target_type = display->getMessageType();
    std::vector<std::string> new_topics;
    if (target_type == "TF") {
        auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(display);
        if (tf_disp) new_topics = tf_disp->getDiscoveredFrames();
    } else if (target_type != "None") {
        auto topic_map = node_->get_topic_names_and_types();
        std::string clean_target = target_type;
        if (clean_target.find("/") == 0) clean_target = clean_target.substr(1);
        for (const auto& [name, types] : topic_map) {
            for (const auto& type : types) {
                std::string clean_type = type; if (clean_type.find("/") == 0) clean_type = clean_type.substr(1);
                if (clean_type == clean_target) { new_topics.push_back(name); break; }
            }
        }
        std::sort(new_topics.begin(), new_topics.end());
    }
    if (new_topics != available_topics_) {
        std::string current = (available_topics_.empty() || topic_idx_ >= (int)available_topics_.size()) ? "" : available_topics_[topic_idx_];
        available_topics_ = new_topics;
        auto it = std::find(available_topics_.begin(), available_topics_.end(), current);
        topic_idx_ = (it != available_topics_.end()) ? std::distance(available_topics_.begin(), it) : 0;
    }
}

Element Visualizer::render_frame() {
    auto terminal = ftxui::Terminal::Size();
    int left_width = 30, right_width = 0;
    Element image_panel = filler(), nav2_panel = filler();
    bool nav2_active = false;
    for (auto& display : displays_) {
        if (!display->isEnabled()) continue;
        if (display->getName() == "Image") {
            auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(display);
            if (img_disp && img_disp->getEnabledTopicCount() > 0) { image_panel = img_disp->render_2d(); right_width = 64; }
        } else if (display->getName() == "Nav2") { nav2_panel = display->render_2d(); nav2_active = true; right_width = 64; }
    }
    if (terminal.dimx < 120) right_width = std::min(right_width, terminal.dimx / 3);

    const int target_height = std::max(10, terminal.dimy - 8);
    const int target_width = std::max(10, terminal.dimx - left_width - right_width - 6);
    const int sw = target_width * 2, sh = target_height * 4;
    
    auto c = Canvas(sw, sh);
    
    // --- Smooth Animation ---
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
    renderer_.clear();
    
    if (grid_display_) grid_display_->render(renderer_, c, fixed_frame_, tf_buffer_);
    for (auto& display : displays_) display->render(renderer_, c, fixed_frame_, tf_buffer_);
    renderer_.finish(c);

    // Update dynamic offsets for mouse picking (left_width=30 + 2 for border/padding, top=2)
    canvas_x_offset_ = left_width + 2;
    canvas_y_offset_ = 2;

    Elements display_list;
    for (size_t i = 0; i < displays_.size(); ++i) {
        int key = (i == 9) ? 0 : (i + 1);
        bool selected = (static_cast<int>(i) == plugin_idx_);
        auto t = text(" [" + std::to_string(key) + "] " + displays_[i]->getName() + (displays_[i]->isEnabled() ? " [X]" : " [ ]"));
        display_list.push_back(selected ? (t | inverted | color(Color::Yellow) | focus) : (t | color(Color::Green)));
    }
    Elements frame_list;
    for (size_t i = 0; i < available_frames_.size(); ++i) {
        bool is_selected = (static_cast<int>(i) == frame_idx_);
        auto t = text(available_frames_[i]);
        frame_list.push_back(is_selected ? (t | inverted | color(Color::Cyan) | focus) : t);
    }
    Elements topic_list;
    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
        auto disp = displays_[plugin_idx_];
        std::string type = disp->getMessageType();
        if (type == "None") topic_list.push_back(text("No settings") | dim);
        else {
            std::string label = (type == "TF") ? "Frames [T/Y]:" : ("Type: " + type);
            topic_list.push_back(text(label) | dim | size(WIDTH, LESS_THAN, 25));
            topic_list.push_back(separator());
            for (size_t i = 0; i < available_topics_.size(); ++i) {
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
                    auto t = text((enabled ? "[X] " : "[ ] ") + name);
                    topic_list.push_back(is_selected ? (t | inverted | color(Color::Magenta) | focus) : t);
                } else topic_list.push_back(is_selected ? (text(name) | inverted | color(Color::Magenta) | focus) : text(name));
            }
        }
    } else topic_list.push_back(text("Select a plugin (Tab)") | dim);

    return vbox({
        hbox({
            text(" TERMINAL RVIZ ") | bold | color(Color::Yellow),
            separator(),
            text(" Frame: " + fixed_frame_) | color(Color::Cyan),
            filler(),
            text(" [V] Tool | [R] Reset | [M] TopDown | [G] Grid | [Tab] Select | [Space] Toggle | [Q] Quit ") | dim
        }),
        separator(),
        hbox({
            vbox({
                text(" ACTIVE TOOL [V] ") | bold | color(Color::Yellow),
                text(" > " + get_tool_name()) | color(Color::White) | border | hcenter,
                text(" PLUGINS ") | bold | color(Color::Yellow),
                vbox(std::move(display_list)) | border,
                text(" FIXED FRAME [F] ") | bold | color(Color::Yellow),
                vbox(std::move(frame_list)) | border | vscroll_indicator | frame | size(HEIGHT, LESS_THAN, 8),
                text(" TOPICS/FRAMES [T/Y] ") | bold | color(Color::Yellow),
                vbox(std::move(topic_list)) | border | vscroll_indicator | frame | size(HEIGHT, EQUAL, 10),
                filler(),
                text(" STATUS: ") | bold,
                text(status_msg_) | color(Color::Green) | size(HEIGHT, EQUAL, 2),
            }) | size(WIDTH, EQUAL, left_width),
            canvas(std::move(c)) | center | flex | border,
            vbox({ image_panel | flex, nav2_active ? nav2_panel : filler(), }) | size(WIDTH, EQUAL, right_width),
        }) | flex
    }) | border;
}

bool Visualizer::handle_event(Event event) {
    if (event == Event::Character('q') || event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }
    
    // 1. Check Global Confirmation (Enter)
    if (event == Event::Return) {
        for (auto& d : displays_) if (d->getName() == "Nav2" && d->isEnabled()) { if (d->handle_event(event)) return true; }
    }

    // 2. Try global Nav2 shortcuts (C, Backspace) if enabled
    for (auto& d : displays_) {
        if (d->getName() == "Nav2" && d->isEnabled()) {
            if (event == Event::Character('c') || event == Event::Character('C') ||
                event == Event::Special("\x7F") || event == Event::Backspace) {
                if (d->handle_event(event)) return true;
            }
        }
    }

    // 3. Delegate to active plugin (Handle Space here for contextual toggle)
    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
        if (displays_[plugin_idx_]->handle_event(event)) return true;
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
    if (event == Event::Character('m') || event == Event::Character('M')) {
        float cx = 0.0f, cy = 0.0f, zoom = 100.0f;
        
        // 1. Find Map Display and get its raw bounds
        for (auto& d : displays_) {
            if (d->getName() == "Map") {
                auto md = std::dynamic_pointer_cast<MapDisplay>(d);
                if (md) {
                    float raw_cx = md->getCenterX(), raw_cy = md->getCenterY();
                    float mw = md->getWidth(), mh = md->getHeight();
                    
                    // Transform raw map center (in 'map' frame) to current fixed_frame_
                    try {
                        auto t = tf_buffer_->lookupTransform(fixed_frame_, "map", tf2::TimePointZero);
                        tf2::Vector3 map_origin(raw_cx, raw_cy, 0.0f);
                        tf2::Transform tf; tf2::fromMsg(t.transform, tf);
                        tf2::Vector3 target = tf * map_origin;
                        cx = target.x(); cy = target.y();
                    } catch (...) {
                        cx = raw_cx; cy = raw_cy; // Fallback
                    }
                    
                    float max_dim = std::max(mw, mh);
                    if (max_dim > 0.1f) zoom = 1500.0f / max_dim; 
                }
                break;
            }
        }
        tar_pitch_ = 1.57f; tar_yaw_ = 0.0f; tar_dist_ = 10.0f;
        tar_cam_x_ = cx; tar_cam_y_ = cy; tar_cam_z_ = 0.0f; tar_zoom_ = zoom;
        return true;
    }
    if (event == Event::Character('+') || event == Event::Character('=')) { tar_zoom_ = tar_zoom_.load() * 1.1f; return true; }
    if (event == Event::Character('-') || event == Event::Character('_')) { tar_zoom_ = tar_zoom_.load() / 1.1f; return true; }
    if (event == Event::Character('g') || event == Event::Character('G')) { if (grid_display_) grid_display_->toggle(); return true; }
    if (event == Event::Character('v') || event == Event::Character('V')) { cycle_tool(); return true; }
    if (event == Event::Tab) { plugin_idx_ = (plugin_idx_ + 1) % (int)displays_.size(); discover_topics(); return true; }
    if (event == Event::Character('f') || event == Event::Character('F')) {
        if (!available_frames_.empty()) { frame_idx_ = (frame_idx_ + 1) % available_frames_.size(); fixed_frame_ = available_frames_[frame_idx_]; }
        return true;
    }
    if (event == Event::Character('t') || event == Event::Character('T')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ - 1 + available_topics_.size()) % available_topics_.size();
            if (plugin_idx_ >= 0) {
                auto disp = displays_[plugin_idx_]; std::string type = disp->getMessageType();
                if (type != "TF" && type != "sensor_msgs/msg/Image" && type != "None") disp->setTopic(available_topics_[topic_idx_]);
            }
        }
        return true;
    }
    if (event == Event::Character('y') || event == Event::Character('Y')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ + 1) % available_topics_.size();
            if (plugin_idx_ >= 0) {
                auto disp = displays_[plugin_idx_]; std::string type = disp->getMessageType();
                if (type != "TF" && type != "sensor_msgs/msg/Image" && type != "None") disp->setTopic(available_topics_[topic_idx_]);
            }
        }
        return true;
    }
    if (event == Event::Character(' ')) {
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            auto disp = displays_[plugin_idx_];
            if (disp->getMessageType() == "TF" && !available_topics_.empty()) {
                auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(disp);
                if (tf_disp) { tf_disp->toggleFrame(available_topics_[topic_idx_]); return true; }
            } else if (!available_topics_.empty()) { disp->setTopic(available_topics_[topic_idx_]); return true; }
        }
    }
    if (event.is_character() && event.character()[0] >= '0' && event.character()[0] <= '9') {
        int key_val = event.character()[0] - '0'; int idx = (key_val == 0) ? 9 : (key_val - 1);
        if (idx >= 0 && idx < (int)displays_.size()) { displays_[idx]->toggle(); plugin_idx_ = idx; discover_topics(); return true; }
    }
    return false;
}

} // namespace terminal_rviz
