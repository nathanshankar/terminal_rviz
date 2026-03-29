#include "terminal_rviz/displays/odometry_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <cmath>

namespace terminal_rviz {

OdometryDisplay::OdometryDisplay(rclcpp::Node::SharedPtr node)
    : Display("Odometry", node) {}

void OdometryDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "odom");
    setTopic(topic_);
}

void OdometryDisplay::setTopic(const std::string& topic) {
    try {
        sub_.reset();
        topic_ = topic;
        sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic_, 10, std::bind(&OdometryDisplay::callback, this, std::placeholders::_1));
        std::lock_guard<std::mutex> lock(mtx_);
        history_.clear();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Odometry: Failed to subscribe to %s: %s", topic.c_str(), e.what());
    }
}

void OdometryDisplay::callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = msg->pose.pose;
    history_.push_back(ps);
    if (history_.size() > max_history_) {
        history_.pop_front();
    }
}

void OdometryDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<geometry_msgs::msg::PoseStamped> to_render;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& p : history_) to_render.push_back(p);
    }

    auto col = ftxui::Color::Red;

    for (const auto& ps : to_render) {
        tf2::Transform o_to_w;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, ps.header.frame_id, tf2::TimePointZero);
            o_to_w.setOrigin(tf2::Vector3(t_msg.transform.translation.x, t_msg.transform.translation.y, t_msg.transform.translation.z));
            o_to_w.setRotation(tf2::Quaternion(t_msg.transform.rotation.x, t_msg.transform.rotation.y, t_msg.transform.rotation.z, t_msg.transform.rotation.w));
        } catch (...) { continue; }

        tf2::Transform v_to_o;
        v_to_o.setOrigin(tf2::Vector3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
        v_to_o.setRotation(tf2::Quaternion(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w));
        tf2::Transform v_to_w = o_to_w * v_to_o;

        // Draw Arrow in world frame with slight vertical offset
        float z_off = 0.1f;
        tf2::Vector3 base = v_to_w * tf2::Vector3(0, 0, 0);
        tf2::Vector3 tip = v_to_w * tf2::Vector3(0.6, 0, 0); // Slightly longer arrow
        
        renderer.draw_line(base.x(), base.y(), base.z() + z_off, tip.x(), tip.y(), tip.z() + z_off, col);
        
        // Larger head for visibility
        tf2::Vector3 dir = (tip - base).normalized();
        tf2::Vector3 side = v_to_w.getBasis() * tf2::Vector3(0, 1, 0);
        tf2::Vector3 left_wing = tip - dir * 0.2 + side * 0.15;
        tf2::Vector3 right_wing = tip - dir * 0.2 - side * 0.15;
        
        renderer.draw_line(tip.x(), tip.y(), tip.z() + z_off, left_wing.x(), left_wing.y(), left_wing.z() + z_off, col);
        renderer.draw_line(tip.x(), tip.y(), tip.z() + z_off, right_wing.x(), right_wing.y(), right_wing.z() + z_off, col);
    }
}

} // namespace terminal_rviz
