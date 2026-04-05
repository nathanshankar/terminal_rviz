#include "terminal_rviz/displays/robot_model_display.hpp"

namespace terminal_rviz {

RobotModelDisplay::RobotModelDisplay(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : Display("RobotModel", node), tf_buffer_(tf_buffer) {
    config_.alpha = 1.0f;
}

void RobotModelDisplay::onInitialize() {
    setTopic("/robot_description");
}

void RobotModelDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    topic_ = topic;
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local(),
        std::bind(&RobotModelDisplay::callback, this, std::placeholders::_1));
}

void RobotModelDisplay::callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    robot_renderer_.init_urdf(msg->data);
}

void RobotModelDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    robot_renderer_.render(renderer, fixed_frame, tf_buffer, config_.alpha);
}

} // namespace terminal_rviz
