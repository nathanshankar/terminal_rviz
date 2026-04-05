#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

TFDisplay::TFDisplay(rclcpp::Node::SharedPtr node)
    : Display("TF", node) {}

void TFDisplay::onInitialize() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

std::vector<std::string> TFDisplay::getDiscoveredFrames() {
    std::vector<std::string> frames;
    if (tf_buffer_) {
        tf_buffer_->_getFrameStrings(frames);
        std::sort(frames.begin(), frames.end());
    }
    return frames;
}

std::vector<std::string> TFDisplay::getEnabledTopics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return enabled_frames_list_;
}

void TFDisplay::toggleFrame(const std::string& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_frames_list_.begin(), enabled_frames_list_.end(), frame);
    if (it != enabled_frames_list_.end()) {
        enabled_frames_list_.erase(it);
        configs_.erase(frame);
    } else {
        enabled_frames_list_.push_back(frame);
        TopicConfig cfg;
        cfg.size = 1.0f;
        cfg.color_style = "Axis";
        configs_[frame] = cfg;
    }
}

bool TFDisplay::isFrameEnabled(const std::string& frame) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (enabled_frames_list_.empty()) return true;
    return std::find(enabled_frames_list_.begin(), enabled_frames_list_.end(), frame) != enabled_frames_list_.end();
}

TopicConfig TFDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void TFDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void TFDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> /*tf_buffer_in*/) {
    if (!enabled_ || !tf_buffer_) return;

    std::vector<std::string> frames;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (enabled_frames_list_.empty()) tf_buffer_->_getFrameStrings(frames);
        else frames = enabled_frames_list_;
    }

    for (const auto& frame : frames) {
        if (frame == fixed_frame) continue;
        
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (configs_.count(frame)) cfg = configs_[frame];
            else { cfg.size = 1.0f; cfg.alpha = 1.0f; cfg.color_style = "Axis"; }
        }

        try {
            auto transform = tf_buffer_->lookupTransform(fixed_frame, frame, tf2::TimePointZero);
            float tx = transform.transform.translation.x;
            float ty = transform.transform.translation.y;
            float tz = transform.transform.translation.z;

            float axis_len = cfg.size;
            float alpha = cfg.alpha;
            tf2::Transform tf;
            tf2::fromMsg(transform.transform, tf);

            // Render axes based on color style
            if (cfg.color_style == "Flat") {
                uint8_t r = 255, g = 255, b = 255;
                static const uint8_t pr[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
                static const uint8_t pg[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
                static const uint8_t pb[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
                int idx = cfg.color_index % 10;
                r = pr[idx]; g = pg[idx]; b = pb[idx];

                // X
                tf2::Vector3 x_pt = tf * tf2::Vector3(axis_len, 0, 0);
                renderer.draw_line(tx, ty, tz, x_pt.x(), x_pt.y(), x_pt.z(), r, g, b, alpha);
                // Y
                tf2::Vector3 y_pt = tf * tf2::Vector3(0, axis_len, 0);
                renderer.draw_line(tx, ty, tz, y_pt.x(), y_pt.y(), y_pt.z(), r, g, b, alpha);
                // Z
                tf2::Vector3 z_pt = tf * tf2::Vector3(0, 0, axis_len);
                renderer.draw_line(tx, ty, tz, z_pt.x(), z_pt.y(), z_pt.z(), r, g, b, alpha);
            } else {
                // Default "Axis" mode: RGB
                // X (Red)
                tf2::Vector3 x_pt = tf * tf2::Vector3(axis_len, 0, 0);
                renderer.draw_line(tx, ty, tz, x_pt.x(), x_pt.y(), x_pt.z(), (uint8_t)255, (uint8_t)0, (uint8_t)0, alpha);

                // Y (Green)
                tf2::Vector3 y_pt = tf * tf2::Vector3(0, axis_len, 0);
                renderer.draw_line(tx, ty, tz, y_pt.x(), y_pt.y(), y_pt.z(), (uint8_t)0, (uint8_t)255, (uint8_t)0, alpha);

                // Z (Blue)
                tf2::Vector3 z_pt = tf * tf2::Vector3(0, 0, axis_len);
                renderer.draw_line(tx, ty, tz, z_pt.x(), z_pt.y(), z_pt.z(), (uint8_t)0, (uint8_t)0, (uint8_t)255, alpha);
            }

        } catch (...) {
            continue;
        }
    }
}

ftxui::Element TFDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& frame : enabled_frames_list_) {
        if (count >= config_scroll && count < config_scroll + 5) { 
            topics_ui.push_back(ConfigHelper::render_summary(frame, configs_[frame]));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_frames_list_.empty() ? " No frames enabled (select in topics)" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" TF Frame Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool TFDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;
    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12); 
    if (ry < 0 || ry >= 10) return false;
    int item_idx = ry + scroll_offset;
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_frames_list_.size()) return true; 
    return false;
}

} // namespace terminal_rviz
