#ifndef TERMINAL_RVIZ_DISPLAYS_CAMERA_INFO_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_CAMERA_INFO_DISPLAY_HPP_

#include <mutex>
#include <map>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class CameraInfoDisplay : public Display {
public:
    explicit CameraInfoDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override;
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override { return "sensor_msgs/msg/CameraInfo"; }

private:
    void callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string& topic);

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subs_;
    std::map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> latest_msgs_;
    std::map<std::string, TopicConfig> configs_;
    std::vector<std::string> enabled_topics_;
    
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_CAMERA_INFO_DISPLAY_HPP_
