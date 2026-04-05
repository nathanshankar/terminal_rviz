#include "terminal_rviz/displays/motion_planning_display.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
#include "ftxui/dom/elements.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace terminal_rviz {

MotionPlanningDisplay::MotionPlanningDisplay(rclcpp::Node::SharedPtr node)
    : Display("MotionPlanning", node) {
    group_name_ = "manipulator";
    ee_link_ = "tool0";
    target_pose_.orientation.w = 1.0;
}

void MotionPlanningDisplay::onInitialize() {
    srand(time(NULL));
    urdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        std::bind(&MotionPlanningDisplay::robot_description_callback, this, std::placeholders::_1));
        
    srdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description_semantic", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            discovered_groups_.clear();
            discovered_poses_.clear();
            group_joints_.clear();
            named_pose_configs_.clear();
            std::string data = msg->data;
            
            // 1. Robustly extract Group names and their Joints
            size_t pos = 0;
            while ((pos = data.find("<group", pos)) != std::string::npos) {
                // Ensure we only match <group tags, not <group_state
                if (pos + 6 < data.size() && data[pos+6] != ' ' && data[pos+6] != '\n' && data[pos+6] != '\t' && data[pos+6] != '>') {
                    pos += 6; continue;
                }

                size_t tag_end = data.find(">", pos);
                size_t name_attr = data.find("name=\"", pos);
                
                if (name_attr != std::string::npos && name_attr < tag_end) {
                    name_attr += 6;
                    size_t name_val_end = data.find("\"", name_attr);
                    if (name_val_end != std::string::npos) {
                        std::string gname = data.substr(name_attr, name_val_end - name_attr);
                        discovered_groups_.push_back(gname);
                        
                        // Extract joints for this group
                        size_t group_end_tag = data.find("</group>", pos);
                        if (group_end_tag != std::string::npos && group_end_tag > tag_end) {
                            size_t j_search = tag_end;
                            while ((j_search = data.find("<joint ", j_search)) != std::string::npos && j_search < group_end_tag) {
                                size_t jn_attr = data.find("name=\"", j_search);
                                if (jn_attr != std::string::npos) {
                                    jn_attr += 6;
                                    size_t jn_end = data.find("\"", jn_attr);
                                    if (jn_end != std::string::npos) {
                                        group_joints_[gname].push_back(data.substr(jn_attr, jn_end - jn_attr));
                                    }
                                }
                                j_search++;
                            }
                        }
                    }
                }
                pos = (tag_end == std::string::npos) ? (pos + 1) : (tag_end + 1);
            }
            std::sort(discovered_groups_.begin(), discovered_groups_.end());

            // Auto-select the first valid group if current is default or missing
            if (!discovered_groups_.empty()) {
                bool current_is_valid = false;
                for (const auto& g : discovered_groups_) {
                    if (g == group_name_) { current_is_valid = true; break; }
                }
                if (!current_is_valid || group_name_ == "arm_group") {
                    group_name_ = discovered_groups_[0];
                }
            }

            // 2. Robustly extract group_states (Named Poses)
            pos = 0;
            while ((pos = data.find("<group_state ", pos)) != std::string::npos) {
                size_t name_attr = data.find("name=\"", pos);
                size_t group_attr = data.find("group=\"", pos);
                size_t tag_end = data.find(">", pos);
                size_t state_end_tag = data.find("</group_state>", pos);
                
                if (name_attr != std::string::npos && group_attr != std::string::npos && 
                    name_attr < tag_end && group_attr < tag_end) {
                    
                    name_attr += 6;
                    size_t name_val_end = data.find("\"", name_attr);
                    group_attr += 7;
                    size_t group_val_end = data.find("\"", group_attr);
                    
                    if (name_val_end != std::string::npos && group_val_end != std::string::npos) {
                        std::string pose_name = data.substr(name_attr, name_val_end - name_attr);
                        std::string g_name = data.substr(group_attr, group_val_end - group_attr);
                        discovered_poses_[g_name].push_back(pose_name);

                        // Parse joints inside this group_state block
                        if (state_end_tag != std::string::npos && state_end_tag > tag_end) {
                            size_t j_search = tag_end;
                            while ((j_search = data.find("<joint ", j_search)) != std::string::npos && j_search < state_end_tag) {
                                size_t jn_start = data.find("name=\"", j_search);
                                size_t jv_start = data.find("value=\"", j_search);
                                if (jn_start != std::string::npos && jv_start != std::string::npos) {
                                    jn_start += 6;
                                    size_t jn_end = data.find("\"", jn_start);
                                    jv_start += 7;
                                    size_t jv_end = data.find("\"", jv_start);
                                    if (jn_end != std::string::npos && jv_end != std::string::npos) {
                                        std::string j_name = data.substr(jn_start, jn_end - jn_start);
                                        try {
                                            double j_val = std::stod(data.substr(jv_start, jv_end - jv_start));
                                            named_pose_configs_[{g_name, pose_name}][j_name] = j_val;
                                        } catch(...) {}
                                    }
                                }
                                j_search++;
                            }
                        }
                    }
                }
                pos = (state_end_tag == std::string::npos) ? (pos + 1) : (state_end_tag + 1);
            }
        });
        
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MotionPlanningDisplay::joint_state_callback, this, std::placeholders::_1));
        
    traj_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", 10,
        std::bind(&MotionPlanningDisplay::trajectory_callback, this, std::placeholders::_1));
        
    move_group_client_ = rclcpp_action::create_client<MoveGroup>(node_, "move_action");
    ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
    fk_client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk");
}

void MotionPlanningDisplay::robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    robot_renderer_.init_urdf(msg->data);
    
    urdf::Model model;
    if (model.initString(msg->data)) {
        discovered_links_.clear();
        joint_limits_.clear();
        std::string auto_ee = "";
        for (auto const& [name, link] : model.links_) {
            discovered_links_.push_back(name);
            if (link->parent_joint && link->child_joints.empty() && auto_ee.empty()) auto_ee = name;
        }
        for (auto const& [name, joint] : model.joints_) {
            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
                if (joint->limits) {
                    joint_limits_[name] = {joint->limits->lower, joint->limits->upper};
                }
            }
        }
        if (!auto_ee.empty()) ee_link_ = auto_ee;
        std::sort(discovered_links_.begin(), discovered_links_.end());
    }
}

void MotionPlanningDisplay::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_joint_state_ = msg;
    
    // Initialize previous_goal_state_ with the starting pose
    if (previous_goal_state_.empty()) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            previous_goal_state_[msg->name[i]] = msg->position[i];
        }
    }
}

void MotionPlanningDisplay::trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_traj_ = msg;
    
    // Extract last state as goal ghost
    if (!msg->trajectory.empty()) {
        const auto& last_traj = msg->trajectory.back();
        if (!last_traj.joint_trajectory.points.empty()) {
            const auto& last_point = last_traj.joint_trajectory.points.back();
            for (size_t i = 0; i < last_traj.joint_trajectory.joint_names.size(); ++i) {
                goal_state_[last_traj.joint_trajectory.joint_names[i]] = last_point.positions[i];
            }
        }
    }
}

void MotionPlanningDisplay::result_callback(const GoalHandleMoveGroup::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(mtx_);
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: status_ = "Succeeded"; break;
        case rclcpp_action::ResultCode::ABORTED:   status_ = "Aborted"; break;
        case rclcpp_action::ResultCode::CANCELED:  status_ = "Canceled"; break;
        default: status_ = "Idle"; break;
    }
}

void MotionPlanningDisplay::set_goal_pos(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_.position.x = x;
    target_pose_.position.y = y;
    target_pose_.position.z = z;
    has_target_ = true;
    solve_ik();
}

void MotionPlanningDisplay::update_goal_relative(float dx, float dy, float dz, float dr, float dp, float dyaw) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_target_) return;
    
    goal_name_ = "Interactive";
    target_pose_.position.x += dx;
    target_pose_.position.y += dy;
    target_pose_.position.z += dz;
    
    if (std::abs(dr) > 1e-4 || std::abs(dp) > 1e-4 || std::abs(dyaw) > 1e-4) {
        tf2::Quaternion q_old, q_rot;
        tf2::fromMsg(target_pose_.orientation, q_old);
        q_rot.setRPY(dr, dp, dyaw);
        target_pose_.orientation = tf2::toMsg(q_rot * q_old);
    }
    solve_ik();
}

void MotionPlanningDisplay::send_goal(bool plan_only) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_target_) return;
    
    if (status_ == "Planning..." || status_ == "Executing...") {
        status_ = "Error: Action in Progress";
        return;
    }

    if (!move_group_client_->wait_for_action_server(std::chrono::seconds(1))) {
        status_ = "Error: No Server"; return;
    }

    auto goal_msg = MoveGroup::Goal();
    goal_msg.request.group_name = group_name_;
    goal_msg.request.pipeline_id = planning_pipeline_;
    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.allowed_planning_time = 5.0;
    goal_msg.planning_options.plan_only = plan_only;

    if (goal_name_ != "Interactive" && goal_name_ != "Random Valid") {
        // Send Joint-Space Goal for named poses and "Current"
        moveit_msgs::msg::Constraints joint_constraints;
        for (auto const& [jname, val] : goal_state_) {
            if (joint_limits_.count(jname)) {
                moveit_msgs::msg::JointConstraint jc;
                jc.joint_name = jname;
                jc.position = val;
                jc.tolerance_above = 0.01;
                jc.tolerance_below = 0.01;
                jc.weight = 1.0;
                joint_constraints.joint_constraints.push_back(jc);
            }
        }
        goal_msg.request.goal_constraints.push_back(joint_constraints);
    } else {
        // Send Cartesian Goal
        moveit_msgs::msg::Constraints c;
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = fixed_frame_ == "map" ? "world" : fixed_frame_; 
        pc.link_name = ee_link_;
        shape_msgs::msg::SolidPrimitive sp;
        sp.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sp.dimensions = {0.01};
        pc.constraint_region.primitives.push_back(sp);
        pc.constraint_region.primitive_poses.push_back(target_pose_);
        pc.weight = 1.0;
        c.position_constraints.push_back(pc);

        moveit_msgs::msg::OrientationConstraint oc;
        oc.header.frame_id = fixed_frame_ == "map" ? "world" : fixed_frame_;
        oc.link_name = ee_link_;
        oc.orientation = target_pose_.orientation;
        oc.absolute_x_axis_tolerance = 0.1;
        oc.absolute_y_axis_tolerance = 0.1;
        oc.absolute_z_axis_tolerance = 0.1;
        oc.weight = 1.0;
        c.orientation_constraints.push_back(oc);

        goal_msg.request.goal_constraints.push_back(c);
    }

    auto options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();
    options.result_callback = std::bind(&MotionPlanningDisplay::result_callback, this, std::placeholders::_1);
    move_group_client_->async_send_goal(goal_msg, options);
    status_ = plan_only ? "Planning..." : "Executing...";
}

void MotionPlanningDisplay::execute() {
    send_goal(false);
}

void MotionPlanningDisplay::solve_ik() {
    if (!ik_client_->service_is_ready()) return;
    if (ik_in_flight_) return; // Prevent piling up requests

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = group_name_;
    request->ik_request.ik_link_name = ee_link_;
    
    // Auto-detect planning frame if fixed_frame_ is default
    std::string plan_frame = fixed_frame_;
    if (plan_frame == "map" || plan_frame == "default") plan_frame = "world"; 

    request->ik_request.pose_stamped.header.frame_id = plan_frame; 
    request->ik_request.pose_stamped.header.stamp = node_->now();
    request->ik_request.pose_stamped.pose = target_pose_;
    request->ik_request.avoid_collisions = false; 
    request->ik_request.timeout.sec = 0;
    request->ik_request.timeout.nanosec = 500000000; // 500ms

    // Add seed state if available
    if (latest_joint_state_) {
        request->ik_request.robot_state.joint_state = *latest_joint_state_;
    }

    ik_in_flight_ = true;
    auto result = ik_client_->async_send_request(request, [this, plan_frame](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
        auto response = future.get();
        ik_in_flight_ = false;
        std::lock_guard<std::mutex> lock(mtx_);
        if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            for (size_t i = 0; i < response->solution.joint_state.name.size(); ++i) {
                goal_state_[response->solution.joint_state.name[i]] = response->solution.joint_state.position[i];
            }
            status_ = "IK Success";
        } else {
            std::stringstream ss;
            ss << "IK Fail:" << response->error_code.val << " (" << std::fixed << std::setprecision(2) 
               << target_pose_.position.x << "," << target_pose_.position.y << "," << target_pose_.position.z << ")";
            status_ = ss.str();
        }
    });
}

void MotionPlanningDisplay::cancel() {
    move_group_client_->async_cancel_all_goals();
}

bool MotionPlanningDisplay::handle_event(ftxui::Event event, int /*scroll_offset*/) {
    auto terminal = ftxui::Terminal::Size();
    
    if (event.is_mouse()) {
        auto mouse = event.mouse();
        if (mouse.button == ftxui::Mouse::Left && mouse.motion == ftxui::Mouse::Pressed) {
            const int right_panel_w = 64;
            const int main_panel_h = 13;

            if (show_group_popup_) {
                int modal_w = 50;
                int max_v = 10;
                
                // Construct the correct list to match render_2d exactly
                std::vector<std::string> list;
                if (popup_type_ == 0) list = discovered_groups_;
                else if (popup_type_ == 1) {
                    list = {"Current", "Random Valid", "Previous"};
                    if (discovered_poses_.count(group_name_)) {
                        for (const auto& p : discovered_poses_[group_name_]) list.push_back(p);
                    }
                }
                else if (popup_type_ == 2) list = discovered_pipelines_;
                int list_size = (int)list.size();
                int items_h = std::max(1, std::min(list_size, max_v));
                int modal_h = items_h + 6; 

                // Popup is centered within the active right-panel area
                int panel_height = std::max(main_panel_h, modal_h);
                int panel_x_start = terminal.dimx - right_panel_w - 1;
                int panel_y_start = terminal.dimy - 1 - panel_height; 
                
                int start_x = panel_x_start + (right_panel_w - modal_w) / 2;
                int start_y = panel_y_start + (panel_height - modal_h) / 2;

                if (mouse.x >= start_x && mouse.x < start_x + modal_w) {
                    int ly = mouse.y - (start_y + 3); 
                    if (ly >= 0 && ly < items_h) {
                        int idx = ly + selection_scroll_;
                        if (idx >= 0 && idx < list_size) {
                            selection_idx_ = idx;
                            if (mouse.button == ftxui::Mouse::Left && mouse.motion == ftxui::Mouse::Pressed) {
                                if (popup_type_ == 0) group_name_ = list[selection_idx_];
                                else if (popup_type_ == 1) {
                                    std::string choice = list[idx];
                                    
                                    // Save current PHYSICAL robot state to "Previous" before changing goal
                                    if (latest_joint_state_ && choice != "Previous") {
                                        std::lock_guard<std::mutex> lock(mtx_);
                                        for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
                                            previous_goal_state_[latest_joint_state_->name[i]] = latest_joint_state_->position[i];
                                        }
                                    }

                                    goal_name_ = choice;
                                    if (choice == "Current") {
                                        has_target_ = false;
                                        status_ = "Goal: Current State";
                                    } else if (choice == "Previous") {
                                        if (!previous_goal_state_.empty()) {
                                            std::lock_guard<std::mutex> lock(mtx_);
                                            goal_state_ = previous_goal_state_;
                                            has_target_ = true;
                                            status_ = "Goal: Restored Previous";
                                            // Sync 3D marker
                                            if (fk_client_->service_is_ready()) {
                                                auto req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
                                                req->header.frame_id = fixed_frame_ == "map" ? "world" : fixed_frame_;
                                                req->fk_link_names = {ee_link_};
                                                for (auto const& [jname, val] : goal_state_) {
                                                    if (joint_limits_.count(jname)) {
                                                        req->robot_state.joint_state.name.push_back(jname);
                                                        req->robot_state.joint_state.position.push_back(val);
                                                    }
                                                }
                                                auto fut = fk_client_->async_send_request(req);
                                                if (fut.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                                                    auto res = fut.get();
                                                    if (!res->pose_stamped.empty()) target_pose_ = res->pose_stamped[0].pose;
                                                }
                                            }
                                        } else {
                                            status_ = "Error: No History";
                                        }
                                    } else if (choice == "Random Valid") {
                                        status_ = "Sampling Joints...";
                                        std::lock_guard<std::mutex> lock(mtx_);
                                        if (group_joints_.count(group_name_)) {
                                            const auto& joints = group_joints_[group_name_];
                                            for (const auto& jname : joints) {
                                                if (joint_limits_.count(jname)) {
                                                    auto limits = joint_limits_[jname];
                                                    double range = limits.second - limits.first;
                                                    double val = limits.first + (static_cast<double>(rand()) / RAND_MAX) * range;
                                                    goal_state_[jname] = val;
                                                }
                                            }
                                            has_target_ = true; // Use sampled joints as goal ghost
                                            
                                            // Call FK to update the interactive sphere position
                                            if (fk_client_->service_is_ready()) {
                                                auto req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
                                                req->header.frame_id = fixed_frame_ == "map" ? "world" : fixed_frame_;
                                                req->fk_link_names = {ee_link_};
                                                
                                                // Only send joints that actually exist in the URDF
                                                for (const auto& jname : joints) {
                                                    if (joint_limits_.count(jname) && goal_state_.count(jname)) {
                                                        req->robot_state.joint_state.name.push_back(jname);
                                                        req->robot_state.joint_state.position.push_back(goal_state_[jname]);
                                                    }
                                                }
                                                
                                                auto fut = fk_client_->async_send_request(req);
                                                if (fut.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                                                    auto res = fut.get();
                                                    if (!res->pose_stamped.empty()) {
                                                        target_pose_ = res->pose_stamped[0].pose;
                                                    }
                                                }
                                            }
                                            status_ = "Goal: " + choice;
                                        } else {
                                            status_ = "Error: Group Joints Unknown";
                                        }
                                    } else {
                                        status_ = "Goal: " + choice;
                                        // Set joint target for named pose
                                        if (named_pose_configs_.count({group_name_, choice})) {
                                            std::lock_guard<std::mutex> lock(mtx_);
                                            auto config = named_pose_configs_[{group_name_, choice}];
                                            for (auto const& [jname, val] : config) {
                                                goal_state_[jname] = val;
                                            }
                                            has_target_ = true;

                                            // Sync the 3D marker to this named pose
                                            if (fk_client_->service_is_ready()) {
                                                auto req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
                                                req->header.frame_id = fixed_frame_ == "map" ? "world" : fixed_frame_;
                                                req->fk_link_names = {ee_link_};
                                                for (auto const& [jname, val] : config) {
                                                    req->robot_state.joint_state.name.push_back(jname);
                                                    req->robot_state.joint_state.position.push_back(val);
                                                }
                                                auto fut = fk_client_->async_send_request(req);
                                                if (fut.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                                                    auto res = fut.get();
                                                    if (!res->pose_stamped.empty()) {
                                                        target_pose_ = res->pose_stamped[0].pose;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                                else if (popup_type_ == 2) planning_pipeline_ = list[selection_idx_];
                                show_group_popup_ = false;
                                return true;
                            }
                        }
                    }
                    if (mouse.y == start_y + modal_h - 2) {
                        show_group_popup_ = false; return true;
                    }
                }
                return true; 
            } else {
                // Clicking the panel [EDIT] buttons
                int ui_start_y = terminal.dimy - 1 - main_panel_h;
                if (mouse.x > terminal.dimx - 12) {
                    if (mouse.y == ui_start_y + 3) { // Group
                        show_group_popup_ = true; popup_type_ = 0; 
                        selection_idx_ = 0; selection_scroll_ = 0; return true; 
                    }
                    if (mouse.y == ui_start_y + 4) { // EE
                        show_group_popup_ = true; popup_type_ = 1; 
                        selection_idx_ = 0; selection_scroll_ = 0; return true; 
                    }
                    if (mouse.y == ui_start_y + 5) { // Lib
                        show_group_popup_ = true; popup_type_ = 2; 
                        selection_idx_ = 0; selection_scroll_ = 0; return true; 
                    }
                }
            }
        }
        if (show_group_popup_ && (mouse.button == ftxui::Mouse::WheelUp || mouse.button == ftxui::Mouse::WheelDown)) {
            if (mouse.button == ftxui::Mouse::WheelUp) selection_scroll_ = std::max(0, selection_scroll_ - 1);
            else selection_scroll_++;
            return true;
        }
    }

    if (show_group_popup_) {
        std::vector<std::string>* list = (popup_type_ == 0) ? &discovered_groups_ : 
                                       ((popup_type_ == 1) ? &discovered_links_ : &discovered_pipelines_);

        if (event == ftxui::Event::ArrowUp) { selection_idx_ = std::max(0, selection_idx_ - 1); return true; }
        if (event == ftxui::Event::ArrowDown) { selection_idx_ = std::min((int)list->size() - 1, selection_idx_ + 1); return true; }
        if (event == ftxui::Event::Return) {
            if (!list->empty()) {
                if (popup_type_ == 0) group_name_ = (*list)[selection_idx_];
                else if (popup_type_ == 1) ee_link_ = (*list)[selection_idx_];
                else if (popup_type_ == 2) planning_pipeline_ = (*list)[selection_idx_];
            }
            show_group_popup_ = false; return true;
        }
        if (event == ftxui::Event::Escape) { show_group_popup_ = false; return true; }
        return true;
    }

    if (event == ftxui::Event::Character('p') || event == ftxui::Event::Character('P')) { send_goal(true); return true; }
    if (event == ftxui::Event::Character('e') || event == ftxui::Event::Character('E')) { execute(); return true; }
    if (event == ftxui::Event::Character('c') || event == ftxui::Event::Character('C')) { cancel(); return true; }
    if (event == ftxui::Event::Character('r') || event == ftxui::Event::Character('R')) { rotate_mode_ = !rotate_mode_; return true; }
    if (event == ftxui::Event::Character('0')) { has_target_ = false; return true; } 
    return false;
}

void MotionPlanningDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        fixed_frame_ = fixed_frame;
    }
    std::lock_guard<std::mutex> lock(mtx_);

    // 0. Update current EE position to allow "grabbing" the real arm
    geometry_msgs::msg::TransformStamped current_ee_tf;
    try {
        current_ee_tf = tf_buffer->lookupTransform(fixed_frame, ee_link_, tf2::TimePointZero);
        if (!has_target_) {
            target_pose_.position.x = current_ee_tf.transform.translation.x;
            target_pose_.position.y = current_ee_tf.transform.translation.y;
            target_pose_.position.z = current_ee_tf.transform.translation.z;
            target_pose_.orientation = current_ee_tf.transform.rotation;
            has_target_ = true; 
        }
    } catch (...) {
        renderer.draw_text(0, 0, 1.5f, " EE NOT FOUND: " + ee_link_ + " (Press 'L' to change) ", ftxui::Color::Red);
    }

    // 1. Render Current Robot
    robot_renderer_.render(renderer, fixed_frame, tf_buffer, 1.0f);

    // 2. Render Goal State Ghost
    if (!goal_state_.empty()) {
        robot_renderer_.render_state(renderer, fixed_frame, tf_buffer, goal_state_, 0.4f);
    }

    // 3. Render Planned Trajectory
    if (latest_traj_) {
        // Trajectory visualization logic here
    }

    // 4. Render Interactive Marker (Goal Pose)
    if (has_target_) {
        float x = target_pose_.position.x, y = target_pose_.position.y, z = target_pose_.position.z;
        uint8_t cr = 255, cg = 100, cb = 0; // Orange
        if (is_selecting_) { cr = 0; cg = 255; cb = 0; }
        
        renderer.draw_sphere(x, y, z, 0.1f, cr, cg, cb, 0.8f);
        renderer.draw_line(x, y, 0, x, y, z, cr, cg, cb, 0.3f);
        
        // Render Orientation Axes at goal
        tf2::Transform goal_tf; tf2::fromMsg(target_pose_, goal_tf);
        float al = 0.3f;
        tf2::Vector3 ax = goal_tf * tf2::Vector3(al,0,0), ay = goal_tf * tf2::Vector3(0,al,0), az = goal_tf * tf2::Vector3(0,0,al);
        renderer.draw_line(x, y, z, ax.x(), ax.y(), ax.z(), 255, 0, 0, 1.0f);
        renderer.draw_line(x, y, z, ay.x(), ay.y(), ay.z(), 0, 255, 0, 1.0f);
        renderer.draw_line(x, y, z, az.x(), az.y(), az.z(), 0, 100, 255, 1.0f);
    }
}

ftxui::Element MotionPlanningDisplay::render_2d(bool /*nav2_active*/, int /*config_scroll*/) {
    if (!enabled_) return ftxui::filler();
    std::lock_guard<std::mutex> lock(mtx_);
    using namespace ftxui;

    auto info = vbox({
        hbox({ text(" Motion Planning ") | bold | color(Color::Orange1), separator(), text(" [v] Tool | [p] Plan | [e] Exec | [r] Rot ") | dim }),
        separator(),
        hbox({ text("Group:  "), text(group_name_) | color(Color::Cyan) | bold, filler(), text(" [EDIT] ") | color(Color::Yellow) | bold }),
        hbox({ text("Goal:   "), text(goal_name_) | color(Color::Blue) | bold, filler(), text(" [EDIT] ") | color(Color::Yellow) | bold }),
        hbox({ text("Lib:    "), text(planning_pipeline_) | color(Color::Green), filler(), text(" [EDIT] ") | color(Color::Yellow) | bold }),
        hbox({ text("Status: "), text(status_) | color(Color::Green) }),
        hbox({ text("Mode:   "), text(rotate_mode_ ? "ROTATION" : "POSITION") | color(rotate_mode_ ? Color::Magenta : Color::Yellow) }),
        separator(),
        vbox({
            hbox({ text("X: "), text(std::to_string(target_pose_.position.x).substr(0,5)), filler(), text("EE: "), text(ee_link_) | dim }), 
            hbox({ text("Y: "), text(std::to_string(target_pose_.position.y).substr(0,5)), filler(), text("Roll:"), text("...") }),
            hbox({ text("Z: "), text(std::to_string(target_pose_.position.z).substr(0,5)), filler(), text("Pitch:"), text("...") }),
        })
    }) | border | bgcolor(Color::Black);

    if (show_group_popup_) {
        std::string title = " PLANNING GROUP ";
        std::vector<std::string> list; 
        if (popup_type_ == 0) list = discovered_groups_;
        else if (popup_type_ == 1) {
            title = " GOAL STATE ";
            list = {"Current", "Random Valid", "Previous"};
            if (discovered_poses_.count(group_name_)) {
                for (const auto& p : discovered_poses_[group_name_]) list.push_back(p);
            }
        }
        else if (popup_type_ == 2) { title = " PLANNING PIPELINE "; list = discovered_pipelines_; }

        Elements items;
        int max_v = 10;
        int list_size = (int)list.size();
        for (int i = selection_scroll_; i < std::min(list_size, selection_scroll_ + max_v); ++i) {
            bool is_selected = (i == selection_idx_);
            auto t = text(" " + list[i]);
            if (is_selected) {
                t = hbox({ text(" > ") | bold | color(Color::Yellow), t | bold | color(Color::White) }) | inverted | focus;
            } else {
                t = hbox({ text("   "), t | color(Color::GrayLight) });
            }
            items.push_back(t);
        }
        if (list.empty()) items.push_back(text(" No options discovered ") | dim);

        int items_h = std::max(1, std::min(list_size, max_v));
        int modal_h = items_h + 6;
        int panel_h = std::max(13, modal_h);

        auto modal_box = vbox({
            text(" EDIT: " + title) | bold | color(Color::Magenta) | hcenter,
            separator(),
            vbox(std::move(items)) | frame | size(HEIGHT, LESS_THAN, max_v),
            separator(),
            hbox({
                filler(),
                text(" [ DONE ] ") | (selection_idx_ == list_size ? (inverted | color(Color::Green) | focus) : nothing),
                filler(),
            })
        }) | size(WIDTH, EQUAL, 50) | border | bgcolor(Color::Black);

        return dbox({
            filler() | bgcolor(Color::Black), // Solid base layer
            vbox({
                filler(),
                hbox({
                    filler(),
                    modal_box,
                    filler(),
                }),
                filler(),
            })
        }) | size(HEIGHT, EQUAL, panel_h);
    }
    return info;
}

} // namespace terminal_rviz
