#ifndef TERMINAL_RVIZ_VISUALIZER_HPP_
#define TERMINAL_RVIZ_VISUALIZER_HPP_

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <set>
#include <filesystem>
#include <fstream>

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

    void set_use_gpu(bool use) { use_gpu_ = use; }
    bool get_use_gpu() const { return use_gpu_; }

    std::string get_fixed_frame() const { return fixed_frame_; }
    void set_grid_display(std::shared_ptr<Display> display) { grid_display_ = display; }
    std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() const { return tf_buffer_; }
    
    void set_status(const std::string& msg) { status_msg_ = msg; }

    enum class Tool { Nav2, Orbit, Pan, MotionPlanning };
    void cycle_tool();
    std::string get_tool_name() const;

    void save_config(const std::string& path);
    void load_config(const std::string& path);

private:
    void discover_frames();
    void discover_topics();
    void build_topic_tree();
    ftxui::Element render_frame();
    bool handle_event(ftxui::Event event, int mouse_dx = 0);

    void refresh_file_list();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    RvizRenderer renderer_;
    std::vector<std::shared_ptr<Display>> displays_;
    std::shared_ptr<Display> grid_display_; 
    mutable std::recursive_mutex displays_mutex_;
    
    std::string fixed_frame_ = "map";
    std::vector<std::string> available_frames_ = {"map"};
    int frame_idx_ = 0;
    
    int plugin_idx_ = -1; 
    int sidebar_hover_idx_ = -1;
    int plugin_scroll_ = 0;
    int config_scroll_ = 0;
    std::vector<std::string> available_topics_;
    int topic_idx_ = 0;
    int topic_scroll_ = 0;
    std::string status_msg_ = "Ready";
    Tool current_tool_ = Tool::Pan;

    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    int canvas_x_offset_ = 32;
    int canvas_y_offset_ = 2;
    int right_width_ = 0;

    bool show_plugin_modal_ = false;
    int modal_selected_idx_ = 0;
    int modal_scroll_ = 0;
    int modal_tab_idx_ = 0; // 0: Plugins, 1: Panels, 2: Topics
    bool modal_plugin_states_[64] = {false};
    
    struct TopicSelectionEntry {
        std::string label;
        std::string topic;
        int display_idx = -1; // Index in displays_ if it's a plugin type leaf
        int indent = 0;
        bool is_plugin_type = false;
        bool is_expanded = false;
        bool has_children = false;
    };
    std::vector<TopicSelectionEntry> modal_topic_entries_;
    std::map<std::pair<std::string, int>, bool> modal_topic_selections_;
    std::set<std::string> expanded_topic_nodes_;

    std::vector<std::string> plugin_names_;
    std::vector<std::string> panel_names_;

    bool show_frame_modal_ = false;
    int modal_frame_selected_idx_ = 0;
    int modal_frame_scroll_ = 0;

    bool show_topic_modal_ = false;
    int topic_modal_selected_idx_ = 0;
    int topic_modal_scroll_ = 0;
    int topic_modal_x_ = 0;
    int topic_modal_y_ = 0;
    std::shared_ptr<Display> topic_target_display_;
    int topic_target_slot_ = -1;
    std::vector<std::string> topic_modal_list_;

    bool show_config_modal_ = false;
    int config_modal_selected_idx_ = 0;
    std::string config_modal_topic_;
    std::shared_ptr<Display> config_target_display_;
    bool is_dragging_config_ = false;
    bool is_dragging_seek_ = false;
    int drag_captured_idx_ = -1;

    // File Browser
    bool show_file_modal_ = false;
    bool is_save_mode_ = false;
    bool is_dir_picker_ = false;
    std::filesystem::path current_path_ = std::filesystem::current_path();
    std::vector<std::filesystem::directory_entry> file_list_;
    int file_selected_idx_ = 0;
    int file_scroll_ = 0;
    std::string input_filename_ = "";

    ftxui::ScreenInteractive screen_;
    std::atomic<bool> quit_flag_{false};
    bool use_gpu_ = false;

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
