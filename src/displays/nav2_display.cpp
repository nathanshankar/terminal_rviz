#include "terminal_rviz/displays/nav2_display.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

namespace terminal_rviz {

Nav2Display::Nav2Display(rclcpp::Node::SharedPtr node)
    : Display("Nav2", node) {}

void Nav2Display::onInitialize() {
#ifdef HAS_NAV2_ACTIONS
    client_ = rclcpp_action::create_client<NavThroughPoses>(node_, "navigate_through_poses");
#else
    nav_status_ = "Unsupported (Foxy/Kilted)";
#endif
}

#ifdef HAS_NAV2_ACTIONS
void Nav2Display::feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback) {
    std::lock_guard<std::mutex> lock(mtx_);
    dist_remaining_ = feedback->distance_remaining;
    recoveries_ = feedback->number_of_recoveries;
    poses_remaining_ = feedback->number_of_poses_remaining;
    
    int secs = feedback->estimated_time_remaining.sec;
    std::stringstream ss; ss << secs / 60 << "m " << secs % 60 << "s";
    time_remaining_ = ss.str();
    nav_status_ = "Executing";
}

void Nav2Display::result_callback(const GoalHandleNav::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(mtx_);
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: nav_status_ = "Succeeded"; break;
        case rclcpp_action::ResultCode::ABORTED:   nav_status_ = "Aborted"; break;
        case rclcpp_action::ResultCode::CANCELED:  nav_status_ = "Canceled"; break;
        default: nav_status_ = "Idle"; break;
    }
    poses_remaining_ = 0;
    dist_remaining_ = 0;
}
#endif

void Nav2Display::set_goal(float x, float y, const std::string& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_selecting_pose_.header.frame_id = frame;
    current_selecting_pose_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    current_selecting_pose_.pose.position.x = x;
    current_selecting_pose_.pose.position.y = y;
    current_selecting_pose_.pose.position.z = 0.0;
    current_selecting_pose_.pose.orientation.w = 1.0;
    has_selecting_pose_ = true;
}

void Nav2Display::update_goal_orientation(float x, float y) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_selecting_pose_) return;
    float dx = x - current_selecting_pose_.pose.position.x;
    float dy = y - current_selecting_pose_.pose.position.y;
    float yaw = std::atan2(dy, dx);
    current_selecting_pose_.pose.orientation.z = std::sin(yaw / 2.0f);
    current_selecting_pose_.pose.orientation.w = std::cos(yaw / 2.0f);
}

void Nav2Display::send_goal() {
    std::lock_guard<std::mutex> lock(mtx_);
#ifdef HAS_NAV2_ACTIONS
    if (!waypoint_queue_.empty()) {
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            nav_status_ = "Err: No Server";
            return;
        }

        auto goal_msg = NavThroughPoses::Goal();
        goal_msg.poses = waypoint_queue_;

        auto send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&Nav2Display::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&Nav2Display::result_callback, this, std::placeholders::_1);
        
        client_->async_send_goal(goal_msg, send_goal_options);
        waypoint_queue_.clear();
        nav_status_ = "Sent";
    }
#else
    nav_status_ = "Actions Unavailable";
#endif
}

void Nav2Display::cancel_nav() {
    std::lock_guard<std::mutex> lock(mtx_);
#ifdef HAS_NAV2_ACTIONS
    client_->async_cancel_all_goals();
#endif
    waypoint_queue_.clear();
    has_selecting_pose_ = false;
    nav_status_ = "Canceling...";
}

void Nav2Display::remove_last() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!waypoint_queue_.empty()) waypoint_queue_.pop_back();
    else has_selecting_pose_ = false;
}

bool Nav2Display::handle_event(ftxui::Event event, int /*scroll_offset*/) {
    if (event == ftxui::Event::Character('c') || event == ftxui::Event::Character('C')) { cancel_nav(); return true; }
    if (event == ftxui::Event::Character(' ') || event == ftxui::Event::Return) { send_goal(); return true; }
    if (event == ftxui::Event::Special("\x7F") || event == ftxui::Event::Backspace) { 
        remove_last(); return true; 
    }
    return false;
}

void Nav2Display::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mtx_);

    auto draw_arrow = [&](const geometry_msgs::msg::Pose& pose, ftxui::Color col) {
        float gx = pose.position.x, gy = pose.position.y, gz = 0.5f;
        double qz = pose.orientation.z, qw = pose.orientation.w;
        double yaw = 2.0 * std::atan2(qz, qw);
        float arrow_len = 0.8f;
        float ex = gx + arrow_len * std::cos(yaw), ey = gy + arrow_len * std::sin(yaw);
        renderer.draw_line(gx, gy, gz, ex, ey, gz, col);
        float ha = 0.4f, hl = 0.25f;
        renderer.draw_line(ex, ey, gz, ex - hl * std::cos(yaw - ha), ey - hl * std::sin(yaw - ha), gz, col);
        renderer.draw_line(ex, ey, gz, ex - hl * std::cos(yaw + ha), ey - hl * std::sin(yaw + ha), gz, col);
        renderer.draw_line(gx, gy, 0, gx, gy, gz, col);
    };

    for (const auto& p : waypoint_queue_) draw_arrow(p.pose, ftxui::Color::Yellow);
    if (has_selecting_pose_) draw_arrow(current_selecting_pose_.pose, is_selecting_ ? ftxui::Color::Green : ftxui::Color::White);
}

void Nav2Display::finalize_selection() {
    std::lock_guard<std::mutex> lock(mtx_);
    is_selecting_ = false;
    if (has_selecting_pose_) {
        waypoint_queue_.push_back(current_selecting_pose_);
        has_selecting_pose_ = false;
    }
}

ftxui::Element Nav2Display::render_2d(bool /*nav2_active*/, int /*config_scroll*/) {
    if (!enabled_) return ftxui::filler();
    std::lock_guard<std::mutex> lock(mtx_);
    using namespace ftxui;
    return vbox({
        hbox({
            text(" Nav2 ") | bold | color(Color::Yellow),
            separator(),
            text(" [Drg] Waypt | [Bksp] Undo | [Ent] Send | [C] Can ") | dim
        }),
        separator(),
        hbox({ text("Status:     "), text(nav_status_) | color(Color::Green) }),
        hbox({ text("Queue Size: "), text(std::to_string(waypoint_queue_.size())) | bold | color(Color::Cyan) }),
        hbox({ text("Poses Left: "), text(std::to_string(poses_remaining_)) | color(Color::White) }),
        separator(),
        hbox({ text("Dist Rem:   "), text(std::to_string(dist_remaining_).substr(0,4) + " m") }),
        hbox({ text("ETA:        "), text(time_remaining_) }),
        hbox({ text("Recoveries: "), text(std::to_string(recoveries_)) | color(Color::Red) }),
    }) | border;
}

} // namespace terminal_rviz
