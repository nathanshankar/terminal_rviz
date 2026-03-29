#ifndef TERMINAL_RVIZ_VISUALIZER_HPP_
#define TERMINAL_RVIZ_VISUALIZER_HPP_

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/dom/elements.hpp"
#include "terminal_rviz/renderer.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class Visualizer {
public:
    Visualizer(rclcpp::Node::SharedPtr node);

    void add_display(std::shared_ptr<Display> display);
    void run();
    void stop();

    std::string get_fixed_frame() const { return fixed_frame_; }
    void set_grid_display(std::shared_ptr<Display> display) { grid_display_ = display; }
    std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() const { return tf_buffer_; }
    
    void set_status(const std::string& msg) { status_msg_ = msg; }

    enum class Tool { Nav2, Orbit, Pan };
    void cycle_tool();
    std::string get_tool_name() const;

private:
    void discover_frames();
    void discover_topics();
    ftxui::Element render_frame();
    bool handle_event(ftxui::Event event);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    RvizRenderer renderer_;
    std::vector<std::shared_ptr<Display>> displays_;
    std::shared_ptr<Display> grid_display_; 
    
    std::string fixed_frame_ = "map";
    std::vector<std::string> available_frames_ = {"map"};
    int frame_idx_ = 0;
    
    int plugin_idx_ = -1; 
    std::vector<std::string> available_topics_;
    int topic_idx_ = 0;
    std::string status_msg_ = "Ready";
    Tool current_tool_ = Tool::Nav2;

    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    int canvas_x_offset_ = 32;
    int canvas_y_offset_ = 2;

    ftxui::ScreenInteractive screen_;
    std::atomic<bool> quit_flag_{false};

    // --- Animated Camera State ---
    // Targets (Atomic for thread safety)
    std::atomic<float> tar_yaw_{0.0f};
    std::atomic<float> tar_pitch_{0.5f};
    std::atomic<float> tar_dist_{5.0f};
    std::atomic<float> tar_cam_x_{0.0f}, tar_cam_y_{0.0f}, tar_cam_z_{0.0f};
    std::atomic<float> tar_zoom_{250.0f};

    // Current interpolation values
    float cur_yaw_ = 0.0f;
    float cur_pitch_ = 0.5f;
    float cur_dist_ = 5.0f;
    float cur_cam_x_ = 0.0f, cur_cam_y_ = 0.0f, cur_cam_z_ = 0.0f;
    float cur_zoom_ = 250.0f;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_VISUALIZER_HPP_
