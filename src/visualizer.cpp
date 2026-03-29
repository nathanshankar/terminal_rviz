#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"

#include <chrono>
#include <set>
#include <map>
#include <algorithm>
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

void Visualizer::run() {
    auto main_renderer = Renderer([this]() { return render_frame(); });
    auto component = CatchEvent(main_renderer, [this](Event event) { return handle_event(event); });

    std::thread ui_thread([&]() {
        while (!quit_flag_ && rclcpp::ok()) {
            discover_frames();
            discover_topics();
            screen_.PostEvent(Event::Custom);
            std::this_thread::sleep_for(100ms);
        }
        screen_.Exit();
    });

    screen_.Loop(component);
    quit_flag_ = true;
    if (ui_thread.joinable()) ui_thread.join();
}

void Visualizer::stop() {
    quit_flag_ = true;
    screen_.Exit();
}

void Visualizer::discover_frames() {
    if (!tf_buffer_) return;
    std::vector<std::string> frames;
    tf_buffer_->_getFrameStrings(frames);
    
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
    if (plugin_idx_ < 0 || static_cast<size_t>(plugin_idx_) >= displays_.size()) {
        available_topics_.clear();
        return;
    }

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
                std::string clean_type = type;
                if (clean_type.find("/") == 0) clean_type = clean_type.substr(1);
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
    int left_panel_width = 30, right_panel_width = 0;
    Elements right_panel;
    for (auto& display : displays_) {
        auto e = display->render_2d();
        if (e && display->isEnabled()) {
            right_panel.push_back(e);
            if (display->getName() == "Image") {
                auto img_disp = std::dynamic_pointer_cast<ImageDisplay>(display);
                if (img_disp && img_disp->getEnabledTopicCount() > 0) right_panel_width = 64; 
            }
        }
    }

    const int target_height = std::max(10, terminal.dimy - 8);
    const int target_width = std::max(10, terminal.dimx - left_panel_width - right_panel_width - 6);
    const int sw = target_width * 2, sh = target_height * 4;
    auto c = Canvas(sw, sh);
    cur_yaw_ += (tar_yaw_.load() - cur_yaw_) * 0.2f;
    cur_pitch_ += (tar_pitch_.load() - cur_pitch_) * 0.2f;
    cur_dist_ += (tar_dist_.load() - cur_dist_) * 0.2f;
    renderer_.set_size(sw, sh);
    renderer_.set_camera(cur_yaw_, cur_pitch_, 0.0f, cur_dist_, cam_x_.load(), cam_y_.load(), cam_z_.load());
    renderer_.set_zoom(zoom_.load());
    renderer_.clear();
    if (grid_display_) grid_display_->render(renderer_, c, fixed_frame_, tf_buffer_);
    for (auto& display : displays_) display->render(renderer_, c, fixed_frame_, tf_buffer_);
    renderer_.finish(c);

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
        if (type == "None") topic_list.push_back(text("No settings for this plugin") | dim);
        else {
            std::string label = (type == "TF") ? "Frames [T/Y to cycle]:" : ("Type: " + type);
            topic_list.push_back(text(label) | dim | size(WIDTH, LESS_THAN, 25));
            topic_list.push_back(separator());
            if (available_topics_.empty()) topic_list.push_back(text("Searching...") | dim | color(Color::Red));
            else {
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
                    } else {
                        auto t = text(name);
                        topic_list.push_back(is_selected ? (t | inverted | color(Color::Magenta) | focus) : t);
                    }
                }
            }
        }
    } else topic_list.push_back(text("Select a plugin (Tab)") | dim);

    return vbox({
        hbox({
            text(" TERMINAL RVIZ ") | bold | color(Color::Yellow),
            separator(),
            text(" Frame: " + fixed_frame_) | color(Color::Cyan),
            filler(),
            text(" [Arrows/PgUp/PgDn] Cam | [G] Grid | [1-0] Toggle | [Tab] Select | [T/Y] Up/Down | [F/Space] Frame/Set | [Q] Quit ") | dim
        }),
        separator(),
        hbox({
            vbox({
                text(" PLUGINS ") | bold | color(Color::Yellow),
                vbox(std::move(display_list)) | border,
                text(" FIXED FRAME [F] ") | bold | color(Color::Yellow),
                vbox(std::move(frame_list)) | border | vscroll_indicator | frame | size(HEIGHT, LESS_THAN, 8),
                text(" TOPICS/FRAMES [T/Y] ") | bold | color(Color::Yellow),
                vbox(std::move(topic_list)) | border | vscroll_indicator | frame | size(HEIGHT, EQUAL, 10),
                filler(),
                text(" STATUS: ") | bold,
                text(status_msg_) | color(Color::Green) | size(HEIGHT, EQUAL, 2),
            }) | size(WIDTH, EQUAL, left_panel_width),
            canvas(std::move(c)) | center | flex | border,
            vbox(std::move(right_panel)) | vscroll_indicator | frame | size(WIDTH, GREATER_THAN, 0),
        }) | flex
    }) | border;
}

bool Visualizer::handle_event(Event event) {
    if (event == Event::Character('q') || event == Event::Escape) { quit_flag_ = true; screen_.Exit(); return true; }
    if (event == Event::ArrowUp)    { cam_z_ = cam_z_.load() + 0.5f; return true; }
    if (event == Event::ArrowDown)  { cam_z_ = cam_z_.load() - 0.5f; return true; }
    if (event == Event::ArrowLeft)  { cam_y_ = cam_y_.load() + 0.5f; return true; }
    if (event == Event::ArrowRight) { cam_y_ = cam_y_.load() - 0.5f; return true; }
    if (event == Event::PageUp)     { cam_x_ = cam_x_.load() + 0.5f; return true; }
    if (event == Event::PageDown)   { cam_x_ = cam_x_.load() - 0.5f; return true; }
    if (event == Event::Character('a')) { tar_yaw_ = tar_yaw_.load() - 0.1f; return true; }
    if (event == Event::Character('d')) { tar_yaw_ = tar_yaw_.load() + 0.1f; return true; }
    if (event == Event::Character('w')) { tar_pitch_ = tar_pitch_.load() + 0.1f; return true; }
    if (event == Event::Character('s')) { tar_pitch_ = tar_pitch_.load() - 0.1f; return true; }
    if (event == Event::Character('+') || event == Event::Character('=')) { zoom_ = zoom_.load() * 1.1f; return true; }
    if (event == Event::Character('-') || event == Event::Character('_')) { zoom_ = zoom_.load() / 1.1f; return true; }
    if (event == Event::Character('g') || event == Event::Character('G')) { if (grid_display_) grid_display_->toggle(); return true; }

    if (event == Event::Tab) {
        plugin_idx_++;
        if (plugin_idx_ >= (int)displays_.size()) plugin_idx_ = 0;
        discover_topics();
        return true;
    }

    if (event == Event::Character('f') || event == Event::Character('F')) {
        if (!available_frames_.empty()) {
            frame_idx_ = (frame_idx_ + 1) % available_frames_.size();
            fixed_frame_ = available_frames_[frame_idx_];
        }
        return true;
    }

    if (event == Event::Character('t') || event == Event::Character('T')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ - 1 + available_topics_.size()) % available_topics_.size();
        }
        return true;
    }

    if (event == Event::Character('y') || event == Event::Character('Y')) {
        if (!available_topics_.empty()) {
            topic_idx_ = (topic_idx_ + 1) % available_topics_.size();
        }
        return true;
    }

    if (event == Event::Character(' ') || event == Event::Return) {
        if (plugin_idx_ >= 0 && plugin_idx_ < (int)displays_.size()) {
            auto disp = displays_[plugin_idx_];
            if (disp->getMessageType() == "TF" && !available_topics_.empty()) {
                auto tf_disp = std::dynamic_pointer_cast<TFDisplay>(disp);
                if (tf_disp) { tf_disp->toggleFrame(available_topics_[topic_idx_]); return true; }
            } else if (!available_topics_.empty()) {
                disp->setTopic(available_topics_[topic_idx_]);
                return true;
            }
        }
    }

    if (event.is_character() && event.character()[0] >= '0' && event.character()[0] <= '9') {
        int key_val = event.character()[0] - '0';
        int idx = (key_val == 0) ? 9 : (key_val - 1);
        if (idx >= 0 && idx < (int)displays_.size()) {
            displays_[idx]->toggle();
            plugin_idx_ = idx;
            discover_topics();
            return true;
        }
    }
    return false;
}

} // namespace terminal_rviz
