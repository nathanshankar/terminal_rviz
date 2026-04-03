#include "terminal_rviz/displays/axes_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

AxesDisplay::AxesDisplay(rclcpp::Node::SharedPtr node)
    : Display("Axes", node) {}

void AxesDisplay::onInitialize() {
}

void AxesDisplay::setTopic(const std::string& frame) {
    toggleFrame(frame);
}

void AxesDisplay::toggleFrame(const std::string& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_frames_.begin(), enabled_frames_.end(), frame);
    if (it != enabled_frames_.end()) {
        enabled_frames_.erase(it);
        configs_.erase(frame);
    } else {
        enabled_frames_.push_back(frame);
        TopicConfig cfg;
        cfg.size = 1.0f; // Default axis length
        configs_[frame] = cfg;
    }
}

bool AxesDisplay::isFrameEnabled(const std::string& frame) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_frames_.begin(), enabled_frames_.end(), frame) != enabled_frames_.end();
}

bool AxesDisplay::isTopicEnabled(const std::string& topic) const {
    return isFrameEnabled(topic);
}

TopicConfig AxesDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void AxesDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

std::string AxesDisplay::getMessageType() const {
    return "tf2_msgs/msg/TFMessage";
}

void AxesDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_ || !tf_buffer) return;

    std::vector<std::string> frames;
    TopicConfig base_cfg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        frames = enabled_frames_;
    }

    for (const auto& frame : frames) {
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (configs_.count(frame)) cfg = configs_[frame];
            else continue;
        }

        try {
            auto transform = tf_buffer->lookupTransform(fixed_frame, frame, tf2::TimePointZero);
            float tx = transform.transform.translation.x;
            float ty = transform.transform.translation.y;
            float tz = transform.transform.translation.z;

            float axis_len = cfg.size;
            float alpha = cfg.alpha;
            tf2::Transform tf;
            tf2::fromMsg(transform.transform, tf);

            // X (Red)
            tf2::Vector3 x_pt = tf * tf2::Vector3(axis_len, 0, 0);
            renderer.draw_line(tx, ty, tz, x_pt.x(), x_pt.y(), x_pt.z(), 255, 0, 0, alpha);

            // Y (Green)
            tf2::Vector3 y_pt = tf * tf2::Vector3(0, axis_len, 0);
            renderer.draw_line(tx, ty, tz, y_pt.x(), y_pt.y(), y_pt.z(), 0, 255, 0, alpha);

            // Z (Blue)
            tf2::Vector3 z_pt = tf * tf2::Vector3(0, 0, axis_len);
            renderer.draw_line(tx, ty, tz, z_pt.x(), z_pt.y(), z_pt.z(), 0, 0, 255, alpha);

        } catch (...) {
            continue;
        }
    }
}

ftxui::Element AxesDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& frame : enabled_frames_) {
        if (count >= config_scroll && count < config_scroll + 5) { 
            topics_ui.push_back(ConfigHelper::render_summary(frame, configs_[frame]));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_frames_.empty() ? " No frames enabled" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" Axes Frames ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool AxesDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;
    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12); 
    if (ry < 0 || ry >= 10) return false;
    int item_idx = ry + scroll_offset;
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_frames_.size()) return true; 
    return false;
}

} // namespace terminal_rviz
