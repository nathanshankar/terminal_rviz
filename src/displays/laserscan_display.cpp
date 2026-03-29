#include "terminal_rviz/displays/laserscan_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

void LaserScanDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    sensor_msgs::msg::LaserScan::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    tf2::Transform laser_to_world;
    try {
        auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(transform_msg.transform, laser_to_world);
    } catch (...) { return; }

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (r >= msg->range_min && r <= msg->range_max) {
            tf2::Vector3 p_local(r * std::cos(angle), r * std::sin(angle), 0.0f);
            tf2::Vector3 p_world = laser_to_world * p_local;
            renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), ftxui::Color::Red);
        }
        angle += msg->angle_increment;
    }
}

} // namespace terminal_rviz
