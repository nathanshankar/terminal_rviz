#ifndef TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "actionlib_msgs/msg/goal_id.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class Nav2Display : public Display {
public:
    explicit Nav2Display(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d() override;
    
    std::string getMessageType() const override { return "Nav2"; }

    void set_goal(float x, float y, const std::string& frame);
    void update_goal_orientation(float x, float y);
    void send_goal();
    void cancel_nav();
    void toggle_mode();
    
    void start_selection() { is_selecting_ = true; }
    void finalize_selection() { is_selecting_ = false; }
    bool is_selecting() const { return is_selecting_; }

private:
    void feedback_callback(const std::shared_ptr<nav2_msgs::action::NavigateToPose_FeedbackMessage> msg);
    void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr cancel_pub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr feedback_sub_;
    
    std::mutex mtx_;
    geometry_msgs::msg::PoseStamped pending_goal_;
    bool has_pending_goal_ = false;
    bool is_selecting_ = false;
    float goal_yaw_ = 0.0f;
    
    // Status info
    std::string nav_status_ = "Idle";
    float dist_remaining_ = 0.0f;
    int recoveries_ = 0;
    std::string time_remaining_ = "0s";
    int poses_remaining_ = 0;
    bool waypoint_mode_ = false;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_
