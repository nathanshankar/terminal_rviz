#ifndef TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <csignal>
#include <sys/types.h>

#include "terminal_rviz/display.hpp"

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
    void start_recording();
    void stop_recording();
    bool is_recording() const { return recording_pid_ > 0; }

private:
    std::mutex mtx_;
    std::set<std::string> selected_topics_;
    std::string output_path_;
    pid_t recording_pid_ = -1;
    
    std::string status_msg_ = "Ready";
    int scroll_pos_ = 0;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_ROSBAG_DISPLAY_HPP_
