#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/config_helper.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"
#include "terminal_rviz/displays/nav2_display.hpp"
#include "terminal_rviz/displays/map_display.hpp"
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

            if (handle_event(event, static_cast<int>(dx))) return true;

            if (mouse.button == Mouse::WheelUp)   { tar_zoom_ = tar_zoom_.load() * 1.1f; return true; }
            if (mouse.button == Mouse::WheelDown) { tar_zoom_ = tar_zoom_.load() / 1.1f; return true; }

            bool is_pressed = (mouse.motion == Mouse::Pressed);
            bool is_released = (mouse.motion == Mouse::Released);
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
    auto topic_map = node_->get_topic_names_and_types();
    std::vector<std::string> topics;
    
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
    int left_width = 30, right_width = 0;
    Element image_panel = filler(), nav2_panel = filler(), config_panel = filler();
    bool nav2_active = false;

    // 1. Pre-calculate active status for global layout
    for (auto& d : displays_) {
        if (d->getName() == "Nav2" && d->isAdded() && d->isEnabled()) nav2_active = true;
    }

    bool show_blocking_modal = show_plugin_modal_ || show_frame_modal_;
    bool show_any_modal = show_blocking_modal || show_topic_modal_ || show_config_modal_;

    // 2. Identify what content to show on the right
    bool has_right_content = false;
    for (auto& display : displays_) {
        if (!display->isAdded() || !display->isEnabled()) continue;
        if (display->getName() == "Image") {
            auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(display);
            if (img_disp && img_disp->getEnabledTopicCount() > 0) { 
                has_right_content = true;
                if (!show_any_modal) image_panel = img_disp->render_2d(nav2_active);
            }
        }
    }
    if (nav2_active) {
        has_right_content = true;
        for (auto& d : displays_) {
            if (d->getName() == "Nav2") {
                if (!show_any_modal) nav2_panel = d->render_2d();
                break;
            }
        }
    }

    // Add configuration panel for selected plugin if it's not Image or Nav2
    bool config_active = false;
    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
        auto disp = displays_[plugin_idx_];
        if (disp->getName() != "Image" && disp->getName() != "Nav2" && disp->isAdded() && disp->isEnabled()) {
            has_right_content = true;
            if (!show_any_modal) {
                config_panel = disp->render_2d(nav2_active, config_scroll_);
                config_active = true;
            }
        }
    }

    if (has_right_content) right_width = 64;
    if (terminal.dimx < 120) right_width = std::min(right_width, terminal.dimx / 3);

    const int target_height = std::max(10, terminal.dimy - 8);
    const int target_width = std::max(10, terminal.dimx - left_width - right_width - 6);
    right_width_ = right_width;
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
    
    if (!show_blocking_modal) {
        renderer_.clear();
        if (grid_display_) grid_display_->render(renderer_, c, fixed_frame_, tf_buffer_);
        for (auto& display : displays_) {
            if (display->isAdded() && display->isEnabled()) {
                display->render(renderer_, c, fixed_frame_, tf_buffer_);
            }
        }
        renderer_.finish(c);
    }

    // Update dynamic offsets for mouse picking
    canvas_x_offset_ = left_width + 2;
    canvas_y_offset_ = 4;

    Elements display_list;
    int visible_plugin_count = 0;
    std::vector<int> sidebar_to_display_idx;
    for (size_t i = 0; i < displays_.size(); ++i) {
        if (!displays_[i]->isAdded()) continue;
        
        if (visible_plugin_count >= plugin_scroll_ && visible_plugin_count < plugin_scroll_ + 8) {
            bool selected = (static_cast<int>(i) == plugin_idx_);
            bool enabled = displays_[i]->isEnabled();
            auto t = text(" " + displays_[i]->getName());
            
            if (selected) {
                t = t | inverted | color(enabled ? Color::Yellow : Color::RedLight) | focus;
            } else {
                t = t | color(enabled ? Color::Green : Color::Red);
            }
            display_list.push_back(t);
        }
        sidebar_to_display_idx.push_back(i);
        visible_plugin_count++;
    }
    if (display_list.empty()) {
        display_list.push_back(text(" No plugins added") | dim);
        display_list.push_back(text(" Press [P] to add") | dim);
    }

    Elements topic_list;
    int visible_topic_count = 0;
    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
        auto disp = displays_[plugin_idx_];
        std::string type = disp->getMessageType();
        if (type == "None") topic_list.push_back(text("No settings") | dim);
        else {
            std::string label = (type == "TF") ? "Frames [T/Y]:" : ("Type: " + type);
            topic_list.push_back(text(label) | dim | size(WIDTH, LESS_THAN, 25));
            topic_list.push_back(separator());
            
            // Limit visible topics based on height
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

    auto main_layout = vbox({
        hbox({
            text(" TERMINAL RVIZ ") | bold | color(Color::Yellow),
            separator(),
            text(" Frame: " + fixed_frame_) | color(Color::Cyan),
            filler(),
            text(" [P] Plugins | [F] Frame | [V] Tool | [R] Reset | [M] TopDown | [G] Grid | [Tab] Select | [Space] Toggle | [Q] Quit ") | dim
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
                    text(" TOPICS/FRAMES [T/Y] ") | bold | color(Color::Yellow),
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
                        // Adjust sx, sy from canvas coordinates to screen coordinates
                        // This is tricky because project() returns canvas-relative coordinates
                        // but we need to place the text element.
                        // Canvas size is sw, sh (target_width*2, target_height*4)
                        // We can use filler() and hbox/vbox to position.
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
                return dbox(std::move(label_elements)) | center | flex | border;
                }(),
                vbox({ 
                (show_topic_modal_ || show_config_modal_ ? (filler() | bgcolor(Color::Black)) : image_panel) | flex, 
                (show_topic_modal_ || show_config_modal_ ? (filler() | size(HEIGHT, EQUAL, 14) | bgcolor(Color::Black)) : 
                    (config_active ? config_panel : (nav2_active ? nav2_panel : filler()))) 
                }) | size(WIDTH, EQUAL, right_width),
                }) | flex
                });
    Element modal_box = filler();

    if (show_plugin_modal_) {
        Elements modal_items;
        int max_visible = 15;
        int display_count = (int)displays_.size();
        
        // Ensure selected index is visible
        if (modal_selected_idx_ < display_count) {
            if (modal_selected_idx_ < modal_scroll_) modal_scroll_ = modal_selected_idx_;
            if (modal_selected_idx_ >= modal_scroll_ + max_visible) modal_scroll_ = modal_selected_idx_ - max_visible + 1;
        }

        for (int i = modal_scroll_; i < std::min(display_count, modal_scroll_ + max_visible); ++i) {
            bool selected = (i == modal_selected_idx_);
            bool checked = modal_plugin_states_[i];
            auto t = text((checked ? " [X] " : " [ ] ") + displays_[i]->getName());
            if (selected) t = t | inverted | focus;
            else t = t | color(checked ? Color::Green : Color::GrayDark);
            modal_items.push_back(t);
        }

        modal_box = vbox({
            text(" PLUGIN SELECTION ") | bold | hcenter | color(Color::Yellow),
            separator(),
            vbox(std::move(modal_items)) | size(HEIGHT, EQUAL, max_visible) | border,
            hbox({
                filler(),
                text(" [ OK ] ") | (modal_selected_idx_ == display_count ? (inverted | color(Color::Green) | focus) : nothing),
                text(" [ CANCEL ] ") | (modal_selected_idx_ == display_count + 1 ? (inverted | color(Color::Red) | focus) : nothing),
                filler(),
            })
        }) | size(WIDTH, GREATER_THAN, 40) | border | bgcolor(Color::Black) | center;
    } else if (show_frame_modal_) {
        show_any_modal = true;
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
        show_any_modal = true;
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

        // Position modal centered on the click slot
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
    if (event == Event::Character('q') || event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }
    
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
            int items_height = std::min(15, frame_count);
            int modal_height = items_height + 7; 
            int start_x = (terminal.dimx - modal_width) / 2;
            int start_y = (terminal.dimy - modal_height) / 2;

            if (mouse.x >= start_x && mouse.x < start_x + modal_width &&
                mouse.y >= start_y && mouse.y < start_y + modal_height) {
                
                int relative_y = mouse.y - (start_y + 4); // Adjust for outer border, header, separator, and inner border
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
        if (event.is_mouse()) {
            auto mouse = event.mouse();
            if (mouse.button == Mouse::WheelUp) { modal_scroll_ = std::max(0, modal_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
            if (mouse.button == Mouse::WheelDown) { 
                int max_scroll = std::max(0, (int)displays_.size() - 15);
                modal_scroll_ = std::min(max_scroll, modal_scroll_ + 1); 
                screen_.PostEvent(Event::Custom); return true; 
            }

            auto terminal = ftxui::Terminal::Size();
            int modal_width = 44; 
            int display_count = (int)displays_.size();
            int items_height = std::min(15, display_count);
            int modal_height = items_height + 7; 
            int start_x = (terminal.dimx - modal_width) / 2;
            int start_y = (terminal.dimy - modal_height) / 2;

            if (mouse.x >= start_x && mouse.x < start_x + modal_width &&
                mouse.y >= start_y && mouse.y < start_y + modal_height) {
                
                int relative_y = mouse.y - (start_y + 4); // Adjust for outer border, header, separator, and inner border
                if (relative_y >= 0 && relative_y < items_height) {
                    int hovered_idx = relative_y + modal_scroll_;
                    if (hovered_idx < display_count) {
                        modal_selected_idx_ = hovered_idx;
                        if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                            modal_plugin_states_[modal_selected_idx_] = !modal_plugin_states_[modal_selected_idx_];
                        }
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }

                if (mouse.y == start_y + modal_height - 2) {
                    int middle = start_x + modal_width / 2;
                    if (mouse.x < middle) modal_selected_idx_ = display_count;
                    else modal_selected_idx_ = display_count + 1;
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
        if (event == Event::ArrowDown) { modal_selected_idx_ = std::min((int)displays_.size() + 1, modal_selected_idx_ + 1); return true; }
        if (event == Event::Character(' ')) {
            if (modal_selected_idx_ < (int)displays_.size()) {
                modal_plugin_states_[modal_selected_idx_] = !modal_plugin_states_[modal_selected_idx_];
            }
            return true;
        }
        if (event == Event::Return) {
            if (modal_selected_idx_ == (int)displays_.size()) { // OK
                for (size_t i = 0; i < displays_.size(); ++i) {
                    if (i < 64) {
                        bool was_added = displays_[i]->isAdded();
                        bool now_added = modal_plugin_states_[i];
                        displays_[i]->setAdded(now_added);
                        if (!was_added && now_added) displays_[i]->setEnabled(true);
                    }
                }
                show_plugin_modal_ = false;
                if (plugin_idx_ < 0 || (plugin_idx_ < (int)displays_.size() && !displays_[plugin_idx_]->isAdded())) {
                    plugin_idx_ = -1;
                    for (size_t i = 0; i < displays_.size(); ++i) if (displays_[i]->isAdded()) { plugin_idx_ = (int)i; break; }
                }
                discover_topics();
            } else if (modal_selected_idx_ == (int)displays_.size() + 1) { // Cancel
                show_plugin_modal_ = false;
            } else if (modal_selected_idx_ < (int)displays_.size()) {
                modal_plugin_states_[modal_selected_idx_] = !modal_plugin_states_[modal_selected_idx_];
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
                
                int relative_y = mouse.y - (start_y + 4); // Adjust for outer border, header, separator, and inner border
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

        auto cfg = config_target_display_->getTopicConfig(config_modal_topic_);
        if (ConfigHelper::handle_edit_event(event, cfg, config_modal_selected_idx_,
                                          show_config_modal_, right_width_, 
                                          config_target_display_->getName(),
                                          mouse_dx,
                                          was_dragging)) {

            config_target_display_->setTopicConfig(config_modal_topic_, cfg);
            screen_.PostEvent(Event::Custom);
            if (!show_config_modal_) is_dragging_config_ = false; // Reset if DONE clicked
            return true;
        }
        return true;
    }

    if (event.is_mouse()) {
        auto mouse = event.mouse();
        auto terminal = ftxui::Terminal::Size();
        if (mouse.x < 31) {
            int y_cursor = 3; 

            // 1. Fixed Frame Section
            if (mouse.y >= y_cursor && mouse.y < y_cursor + 6) {
                if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                    show_frame_modal_ = true;
                    modal_frame_selected_idx_ = frame_idx_;
                    screen_.PostEvent(Event::Custom);
                }
                return true;
            }
            y_cursor += 6;

            // 2. Tool Section
            if (mouse.y >= y_cursor && mouse.y < y_cursor + 6) {
                if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                    cycle_tool();
                    screen_.PostEvent(Event::Custom);
                }
                return true;
            }
            y_cursor += 6;

            // 3. Plugins Section
            int plugin_box_height = 13;
            if (mouse.y >= y_cursor && mouse.y < y_cursor + plugin_box_height) {
                if (mouse.button == Mouse::WheelUp)   { plugin_scroll_ = std::max(0, plugin_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                if (mouse.button == Mouse::WheelDown) { plugin_scroll_++; screen_.PostEvent(Event::Custom); return true; }
                
                if (mouse.y == y_cursor + 1) {
                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                        show_plugin_modal_ = true;
                        modal_selected_idx_ = 0;
                        for (size_t i = 0; i < displays_.size(); ++i) if (i < 64) modal_plugin_states_[i] = displays_[i]->isAdded();
                    }
                    screen_.PostEvent(Event::Custom);
                    return true;
                }
                int list_y = mouse.y - (y_cursor + 3) + plugin_scroll_;
                if (list_y >= 0) {
                    // Find the Nth "Added" plugin
                    int count = 0;
                    for (size_t i = 0; i < displays_.size(); ++i) {
                        if (displays_[i]->isAdded()) {
                            if (count == list_y) {
                                plugin_idx_ = i;
                                if (mouse.button == Mouse::Right && mouse.motion == Mouse::Pressed) {
                                    displays_[i]->setEnabled(!displays_[i]->isEnabled());
                                }
                                discover_topics();
                                screen_.PostEvent(Event::Custom);
                                return true;
                            }
                            count++;
                        }
                    }
                }
                return true;
            }
            y_cursor += plugin_box_height;

            // 4. Topics Section
            if (mouse.y >= y_cursor && mouse.y < ftxui::Terminal::Size().dimy - 4) {
                if (mouse.button == Mouse::WheelUp)   { topic_scroll_ = std::max(0, topic_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                if (mouse.button == Mouse::WheelDown) { topic_scroll_++; screen_.PostEvent(Event::Custom); return true; }
                
                int list_y = mouse.y - (y_cursor + 3) + topic_scroll_;
                if (list_y >= 0 && list_y < (int)available_topics_.size()) {
                    topic_idx_ = list_y;
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
            // Right panel click handling
            int ry = mouse.y - 3; 
            if (ry >= 0) {
                // Determine if we are in the bottom area (Config/Nav2) or top area (Images)
                // Bottom area is fixed height 14 lines (Border + Header + Sep + 10 content + Border)
                int bottom_h = 14;
                bool in_bottom = (mouse.y >= terminal.dimy - bottom_h - 1);

                if (!in_bottom) {
                    // Top Area: Image selection
                    std::shared_ptr<ImageDisplay> img_disp = nullptr;
                    bool nav2_active = false;
                    for (auto& d : displays_) {
                        if (d->isAdded() && d->isEnabled()) {
                            if (d->getName() == "Image") img_disp = std::dynamic_pointer_cast<ImageDisplay>(d);
                            if (d->getName() == "Nav2") nav2_active = true;
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
                                if (slot_ry <= 2) { // Clicked title area
                                    if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                                        // Open topic selection modal
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
                    // Bottom Area: Config/Nav2
                    if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
                        auto disp = displays_[plugin_idx_];
                        if (disp->isAdded() && disp->isEnabled()) {
                            if (mouse.button == Mouse::WheelUp)   { config_scroll_ = std::max(0, config_scroll_ - 1); screen_.PostEvent(Event::Custom); return true; }
                            if (mouse.button == Mouse::WheelDown) { config_scroll_++; screen_.PostEvent(Event::Custom); return true; }
                            
                            // Check for click to open config
                            if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
                                int cry = mouse.y - (terminal.dimy - 12); 
                                if (cry >= 0 && cry < 10) {
                                    int item_idx = cry + config_scroll_;
                                    auto topics = disp->getEnabledTopics();
                                    if (item_idx >= 0 && item_idx < (int)topics.size()) {
                                        show_config_modal_ = true;
                                        config_modal_topic_ = topics[item_idx];
                                        config_target_display_ = disp;
                                        config_modal_selected_idx_ = 0;
                                        screen_.PostEvent(Event::Custom);
                                        return true;
                                    }
                                }
                            }
                            if (disp->handle_event(event, config_scroll_)) return true;
                        }
                    }
                }
            }
            // Consume all mouse events in the right panel to prevent visualizer zoom
            return true;
        }
    }

    if (event == Event::Character('p') || event == Event::Character('P')) {
        show_plugin_modal_ = !show_plugin_modal_;
        if (show_plugin_modal_) {
            modal_selected_idx_ = 0;
            for (size_t i = 0; i < displays_.size(); ++i) {
                if (i < 64) modal_plugin_states_[i] = displays_[i]->isAdded();
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

    // 2. Try global Nav2 shortcuts (Enter, C, Backspace) if active (Added and Enabled)
    for (auto& d : displays_) {
        if (d->getName() == "Nav2" && d->isAdded() && d->isEnabled()) {
            if (event == Event::Return ||
                event == Event::Character('c') || event == Event::Character('C') ||
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
    if (event == Event::Tab) {
        int count = 0;
        do {
            plugin_idx_ = (plugin_idx_ + 1) % (int)displays_.size();
            count++;
        } while (!displays_[plugin_idx_]->isEnabled() && count < (int)displays_.size());
        
        if (!displays_[plugin_idx_]->isEnabled()) plugin_idx_ = -1;
        discover_topics();
        return true;
    }
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
    return false;
}

} // namespace terminal_rviz
