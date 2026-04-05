#ifndef TERMINAL_RVIZ_DISPLAYS_MOTION_PLANNING_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_MOTION_PLANNING_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <atomic>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "terminal_rviz/display.hpp"
#include "terminal_rviz/robot_renderer.hpp"

namespace terminal_rviz {

class MotionPlanningDisplay : public Display {
public:
    using MoveGroup = moveit_msgs::action::MoveGroup;
    using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

    explicit MotionPlanningDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;    

    std::string getMessageType() const override { return "MoveIt/MotionPlanning"; }

    // Interactivity
    void set_goal_pos(float x, float y, float z);
    void set_goal_rot(float roll, float pitch, float yaw);
    void update_goal_relative(float dx, float dy, float dz, float dr, float dp, float dyaw);
    
    void send_goal(bool plan_only = true);
    void execute();
    void cancel();
    
    void solve_ik(); // For real-time ghost updates

    void start_selection() { is_selecting_ = true; }
    void finalize_selection() { is_selecting_ = false; }
    bool is_selecting() const { return is_selecting_; }
    bool has_target() const { return has_target_; }
    geometry_msgs::msg::Point get_goal_pos() const { return target_pose_.position; }

private:
    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
    void result_callback(const GoalHandleMoveGroup::WrappedResult& result);

    RobotRenderer robot_renderer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr srdf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr traj_sub_;
    rclcpp_action::Client<MoveGroup>::SharedPtr move_group_client_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;

    std::mutex mtx_;
    std::string fixed_frame_ = "map";
    std::string group_name_ = "arm_group";
    std::string ee_link_ = "tool0";
    std::string planning_pipeline_ = "ompl";
    
    std::vector<std::string> discovered_groups_;
    std::vector<std::string> discovered_links_;
    std::vector<std::string> discovered_pipelines_ = {"ompl", "pilz_industrial_motion_planner", "chomp", "stomp"};
    std::map<std::string, std::vector<std::string>> discovered_poses_; // group -> list of pose names
    std::map<std::string, std::vector<std::string>> group_joints_;    // group -> list of joint names
    std::map<std::string, std::pair<double, double>> joint_limits_;   // joint -> {lower, upper}
    std::map<std::pair<std::string, std::string>, std::map<std::string, double>> named_pose_configs_; // {group, pose} -> {joint -> val}
    
    std::string goal_name_ = "Interactive";
    int selection_idx_ = 0;
    int selection_scroll_ = 0;

    geometry_msgs::msg::Pose target_pose_;
    bool has_target_ = false;
    bool is_selecting_ = false;
    bool rotate_mode_ = false;

    std::map<std::string, double> goal_state_;
    std::map<std::string, double> previous_goal_state_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    moveit_msgs::msg::DisplayTrajectory::SharedPtr latest_traj_;
    std::atomic<bool> ik_in_flight_{false};
    
    std::string status_ = "Idle";
    bool show_group_popup_ = false;
    std::string input_buffer_;
    int popup_type_ = 0; // 0: Group, 1: EE Link, 2: Pipeline
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_MOTION_PLANNING_DISPLAY_HPP_
