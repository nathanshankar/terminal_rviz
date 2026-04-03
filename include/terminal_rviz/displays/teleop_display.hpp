#ifndef TERMINAL_RVIZ_DISPLAYS_TELEOP_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_TELEOP_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <map>

#include "geometry_msgs/msg/twist.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class TeleopDisplay : public Display {
public:
    explicit TeleopDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    
    std::string getMessageType() const override { return "geometry_msgs/msg/Twist"; }
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override { return topic == topic_; }
    std::vector<std::string> getEnabledTopics() const override { return {topic_}; }

private:
    void publish_twist(double linear_x, double angular_z);

    std::mutex mtx_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    
    double speed_ = 0.5;
    double turn_ = 1.0;
    double x_ = 0.0;
    double th_ = 0.0;
    
    std::string last_key_ = "None";
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_TELEOP_DISPLAY_HPP_
