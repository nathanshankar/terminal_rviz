#include "terminal_rviz/displays/odometry_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace terminal_rviz {

OdometryDisplay::OdometryDisplay(rclcpp::Node::SharedPtr node)
    : Display("Odometry", node) {}

void OdometryDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "odom");
    setTopic(topic_);
}

void OdometryDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topic_, 10, std::bind(&OdometryDisplay::callback, this, std::placeholders::_1));
}

void OdometryDisplay::callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    path_.push_back(msg->pose.pose.position);
    if (path_.size() > max_path_size_) {
        path_.pop_front();
    }
}

void OdometryDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::deque<geometry_msgs::msg::Point> path;
    std::string frame_id = "odom"; // Default
    {
        std::lock_guard<std::mutex> lock(mtx_);
        path = path_;
        // Odometry messages usually have a header
    }

    if (path.size() < 2) return;

    // Odometry is already usually in a fixed-ish frame, but let's assume points are in odom frame
    tf2::Transform odom_to_world;
    try {
        // We take the last frame_id we saw or default to odom
        auto transform_msg = tf_buffer->lookupTransform(fixed_frame, "odom", tf2::TimePointZero);
        tf2::fromMsg(transform_msg.transform, odom_to_world);
    } catch (...) { 
        odom_to_world.setIdentity(); 
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        tf2::Vector3 p1_local(path[i].x, path[i].y, path[i].z);
        tf2::Vector3 p2_local(path[i+1].x, path[i+1].y, path[i+1].z);
        tf2::Vector3 p1 = odom_to_world * p1_local;
        tf2::Vector3 p2 = odom_to_world * p2_local;
        renderer.draw_line(
            p1.x(), p1.y(), p1.z(),
            p2.x(), p2.y(), p2.z(),
            ftxui::Color::Yellow, canvas);
    }
}

} // namespace terminal_rviz
