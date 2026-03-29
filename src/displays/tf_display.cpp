#include "terminal_rviz/displays/tf_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

void TFDisplay::toggleFrame(const std::string& frame) {
    if (enabled_frames_.count(frame)) {
        enabled_frames_.erase(frame);
    } else {
        enabled_frames_.insert(frame);
    }
}

bool TFDisplay::isFrameEnabled(const std::string& frame) const {
    // If we haven't toggled anything, default to all frames visible
    if (enabled_frames_.empty()) return true;
    return enabled_frames_.count(frame) > 0;
}

void TFDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_ || !tf_buffer_) return;

    std::vector<std::string> frames;
    tf_buffer_->_getFrameStrings(frames);

    for (const auto& frame : frames) {
        if (frame == fixed_frame) continue;
        if (!isFrameEnabled(frame)) continue;

        try {
            auto transform = tf_buffer_->lookupTransform(fixed_frame, frame, tf2::TimePointZero);
            float tx = transform.transform.translation.x;
            float ty = transform.transform.translation.y;
            float tz = transform.transform.translation.z;

            float axis_len = 0.3f;
            tf2::Transform tf;
            tf2::fromMsg(transform.transform, tf);

            // X (Red)
            tf2::Vector3 x_pt = tf * tf2::Vector3(axis_len, 0, 0);
            renderer.draw_line(tx, ty, tz, x_pt.x(), x_pt.y(), x_pt.z(), ftxui::Color::Red, canvas);

            // Y (Green)
            tf2::Vector3 y_pt = tf * tf2::Vector3(0, axis_len, 0);
            renderer.draw_line(tx, ty, tz, y_pt.x(), y_pt.y(), y_pt.z(), ftxui::Color::Green, canvas);

            // Z (Blue)
            tf2::Vector3 z_pt = tf * tf2::Vector3(0, 0, axis_len);
            renderer.draw_line(tx, ty, tz, z_pt.x(), z_pt.y(), z_pt.z(), ftxui::Color::Blue, canvas);

        } catch (...) {
            continue;
        }
    }
}

} // namespace terminal_rviz
