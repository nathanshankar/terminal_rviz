#ifndef TERMINAL_RVIZ_TEMPERATURE_DISPLAY_HPP_
#define TERMINAL_RVIZ_TEMPERATURE_DISPLAY_HPP_

#include "terminal_rviz/display.hpp"
#include <sensor_msgs/msg/temperature.hpp>
#include <map>
#include <mutex>

namespace terminal_rviz {

class TemperatureDisplay : public Display {
public:
    explicit TemperatureDisplay(rclcpp::Node::SharedPtr node);
    void onInitialize() override {}
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;

    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override { std::lock_guard<std::mutex> lock(mtx_); return enabled_topics_; }
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override { return "sensor_msgs/msg/Temperature"; }

private:
    void callback(const sensor_msgs::msg::Temperature::SharedPtr msg, const std::string& topic);

    mutable std::mutex mtx_;
    std::vector<std::string> enabled_topics_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr> subs_;
    std::map<std::string, sensor_msgs::msg::Temperature::SharedPtr> latest_msgs_;
    std::map<std::string, TopicConfig> configs_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_TEMPERATURE_DISPLAY_HPP_
