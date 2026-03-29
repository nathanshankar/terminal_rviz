#include "terminal_rviz/displays/nav2_display.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

namespace terminal_rviz {

Nav2Display::Nav2Display(rclcpp::Node::SharedPtr node)
    : Display("Nav2", node) {}

void Nav2Display::onInitialize() {
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    cancel_pub_ = node_->create_publisher<actionlib_msgs::msg::GoalID>("navigate_to_pose/_action/cancel_goal", 10);
    status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status", 10, std::bind(&Nav2Display::status_callback, this, std::placeholders::_1));
    feedback_sub_ = node_->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
        "navigate_to_pose/_action/feedback", 10, std::bind(&Nav2Display::feedback_callback, this, std::placeholders::_1));
}

void Nav2Display::status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (msg->status_list.empty()) nav_status_ = "Idle";
    else {
        auto last = msg->status_list.back();
        switch (last.status) {
            case 1: nav_status_ = "Accepted"; break;
            case 2: nav_status_ = "Executing"; break;
            case 4: nav_status_ = "Succeeded"; break;
            case 5: nav_status_ = "Canceled"; break;
            case 6: nav_status_ = "Aborted"; break;
            default: nav_status_ = "Unknown"; break;
        }
    }
}

void Nav2Display::feedback_callback(const std::shared_ptr<nav2_msgs::action::NavigateToPose_FeedbackMessage> msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    dist_remaining_ = msg->feedback.distance_remaining;
    recoveries_ = msg->feedback.number_of_recoveries;
    int secs = msg->feedback.estimated_time_remaining.sec;
    std::stringstream ss; ss << secs / 60 << "m " << secs % 60 << "s";
    time_remaining_ = ss.str();
}

void Nav2Display::set_goal(float x, float y, const std::string& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_goal_.header.frame_id = frame;
    pending_goal_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    pending_goal_.pose.position.x = x;
    pending_goal_.pose.position.y = y;
    pending_goal_.pose.position.z = 0.0;
    goal_yaw_ = 0.0f;
    has_pending_goal_ = true;
}

void Nav2Display::update_goal_orientation(float x, float y) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_pending_goal_) return;
    float dx = x - pending_goal_.pose.position.x;
    float dy = y - pending_goal_.pose.position.y;
    goal_yaw_ = std::atan2(dy, dx);
}

void Nav2Display::send_goal() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_pending_goal_) {
        pending_goal_.pose.orientation.z = std::sin(goal_yaw_ / 2.0f);
        pending_goal_.pose.orientation.w = std::cos(goal_yaw_ / 2.0f);
        goal_pub_->publish(pending_goal_);
        has_pending_goal_ = false;
    }
}

void Nav2Display::cancel_nav() {
    std::lock_guard<std::mutex> lock(mtx_);
    actionlib_msgs::msg::GoalID msg; 
    cancel_pub_->publish(msg);
    has_pending_goal_ = false;
}

void Nav2Display::toggle_mode() {
    std::lock_guard<std::mutex> lock(mtx_);
    waypoint_mode_ = !waypoint_mode_;
}

void Nav2Display::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_pending_goal_) {
        float gx = pending_goal_.pose.position.x, gy = pending_goal_.pose.position.y, gz = 0.1f;
        ftxui::Color col = is_selecting_ ? ftxui::Color::Green : ftxui::Color::Yellow;
        
        // Draw the Arrow
        float arrow_len = 1.0f;
        float ex = gx + arrow_len * std::cos(goal_yaw_);
        float ey = gy + arrow_len * std::sin(goal_yaw_);
        renderer.draw_line(gx, gy, gz, ex, ey, gz, col);
        
        // Arrow head
        float head_angle = 0.4f; float head_len = 0.3f;
        renderer.draw_line(ex, ey, gz, ex - head_len * std::cos(goal_yaw_ - head_angle), ey - head_len * std::sin(goal_yaw_ - head_angle), gz, col);
        renderer.draw_line(ex, ey, gz, ex - head_len * std::cos(goal_yaw_ + head_angle), ey - head_len * std::sin(goal_yaw_ + head_angle), gz, col);
        
        // Vertical line for visibility
        renderer.draw_line(gx, gy, 0, gx, gy, 0.5f, col);
    }
}

ftxui::Element Nav2Display::render_2d() {
    if (!enabled_) return ftxui::filler();
    std::lock_guard<std::mutex> lock(mtx_);
    using namespace ftxui;
    return vbox({
        text(" Nav2 Dashboard ") | bold | color(Color::Yellow),
        separator(),
        hbox({ text("Status: "), text(nav_status_) | color(nav_status_ == "Executing" ? Color::Green : Color::White) }),
        hbox({ text("Mode:   "), text(waypoint_mode_ ? "Waypoints" : "Navigation") }),
        separator(),
        hbox({ text("Dist:   "), text(std::to_string(dist_remaining_).substr(0,4) + "m") }),
        hbox({ text("ETA:    "), text(time_remaining_) }),
        separator(),
        vbox({
            text(" [Click+Drag] Map to set Pose"),
            text(" [Space]      Confirm & Send"),
            text(" [R]          Cancel | [M] Mode")
        }) | dim
    }) | border;
}

} // namespace terminal_rviz
