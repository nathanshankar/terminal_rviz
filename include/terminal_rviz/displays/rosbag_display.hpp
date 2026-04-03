#ifndef TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_

#ifdef HAS_ROSBAG2

#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <csignal>
#include <sys/types.h>

#include "rosgraph_msgs/msg/clock.hpp"
#include "terminal_rviz/display.hpp"
#include "rosbag2_interfaces/srv/seek.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_interfaces/srv/set_rate.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"

namespace terminal_rviz {

class RosbagDisplay : public Display {
public:
    explicit RosbagDisplay(rclcpp::Node::SharedPtr node);
    virtual ~RosbagDisplay();

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    
    std::string getMessageType() const override { return "Rosbag"; }
    
    void toggle_topic(const std::string& topic);
    void set_output_path(const std::string& path);
    void set_input_path(const std::string& path);
    void set_tab(int idx) { tab_idx_ = idx; }
    int get_tab() const { return tab_idx_; }
    
    void set_seeking(bool seeking) { is_seeking_ = seeking; }
    bool is_seeking() const { return is_seeking_; }
    
    void start_scrubbing(float progress);
    void finish_scrubbing();
    
    void toggle_pause();
    void toggle_loop();
    void set_playback_rate(float rate);
    void seek(float progress, bool force = false);
    float get_playback_rate() const { return playback_rate_; }
    float get_progress() const { return current_progress_; }
    bool is_paused() const { return is_paused_; }
    bool is_looping() const { return is_looping_; }

    void start_recording();
    void stop_recording();
    void start_playback(float start_progress = 0.0f, bool start_paused = false);
    void stop_playback();
    bool is_playing() const { return playback_pid_ > 0; }

    private:
    std::mutex mtx_;
    int tab_idx_ = 1; // Default to Play tab
    std::set<std::string> selected_topics_;
    std::string output_path_;
    std::string input_path_;
    float playback_rate_ = 1.0f;
    float current_progress_ = 0.0f;
    bool is_paused_ = false;
    bool is_looping_ = false;
    bool is_seeking_ = false;
    bool was_playing_before_scrub_ = false;
    bool finished_ = false;
    long long bag_duration_ns_ = 0;
    long long bag_start_time_ns_ = 0;

    std::chrono::steady_clock::time_point last_seek_time_;
    std::chrono::steady_clock::time_point last_service_call_time_;
    float last_seek_target_progress_ = -1.0f;

    pid_t recording_pid_ = -1;
    pid_t playback_pid_ = -1;

    std::string status_msg_ = "Ready";
    int scroll_pos_ = 0;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    // Service clients
    rclcpp::Client<rosbag2_interfaces::srv::Seek>::SharedPtr seek_client_;
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_client_;
    rclcpp::Client<rosbag2_interfaces::srv::SetRate>::SharedPtr rate_client_;
    rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr pause_client_;
    rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr resume_client_;
};

} // namespace terminal_rviz

#endif // HAS_ROSBAG2

#endif // TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_
