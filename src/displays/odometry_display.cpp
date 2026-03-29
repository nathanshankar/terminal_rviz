#include "terminal_rviz/displays/odometry_display.hpp"

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

void OdometryDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    std::deque<geometry_msgs::msg::Point> path;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        path = path_;
    }

    if (path.size() < 2) return;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        renderer.draw_line(
            path[i].x, path[i].y, path[i].z,
            path[i+1].x, path[i+1].y, path[i+1].z,
            ftxui::Color::Yellow, canvas);
    }
}

} // namespace terminal_rviz
