#include "terminal_rviz/displays/laserscan_display.hpp"
#include <cmath>

namespace terminal_rviz {

LaserScanDisplay::LaserScanDisplay(rclcpp::Node::SharedPtr node)
    : Display("LaserScan", node) {}

void LaserScanDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "scan");
    setTopic(topic_);
}

void LaserScanDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_, 10, std::bind(&LaserScanDisplay::callback, this, std::placeholders::_1));
}

void LaserScanDisplay::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_msg_ = msg;
}

void LaserScanDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    sensor_msgs::msg::LaserScan::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (r >= msg->range_min && r <= msg->range_max) {
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);
            renderer.draw_point(x, y, 0.0f, ftxui::Color::Red, canvas);
        }
        angle += msg->angle_increment;
    }
}

} // namespace terminal_rviz
