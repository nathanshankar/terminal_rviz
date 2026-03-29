#ifndef TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "actionlib_msgs/msg/goal_id.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class Nav2Display : public Display {
public:
    using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavThroughPoses>;

    explicit Nav2Display(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d() override;
    bool handle_event(ftxui::Event event) override;
    
    std::string getMessageType() const override { return "Nav2"; }

    void set_goal(float x, float y, const std::string& frame);
    void update_goal_orientation(float x, float y);
    void send_goal();
    void cancel_nav();
    void remove_last();
    
    void start_selection() { is_selecting_ = true; }
    void finalize_selection();
    bool is_selecting() const { return is_selecting_; }

private:
    void feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavThroughPoses::Feedback> feedback);
    void result_callback(const GoalHandleNav::WrappedResult& result);

    rclcpp_action::Client<NavThroughPoses>::SharedPtr client_;
    
    std::mutex mtx_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoint_queue_;
    geometry_msgs::msg::PoseStamped current_selecting_pose_;
    
    bool has_selecting_pose_ = false;
    bool is_selecting_ = false;
    float goal_yaw_ = 0.0f;
    
    // Status info
    std::string nav_status_ = "Idle";
    float dist_remaining_ = 0.0f;
    int recoveries_ = 0;
    std::string time_remaining_ = "0s";
    int poses_remaining_ = 0;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_NAV2_DISPLAY_HPP_
