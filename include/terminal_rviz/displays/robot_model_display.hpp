#ifndef TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "terminal_rviz/display.hpp"
#include "terminal_rviz/robot_renderer.hpp"

namespace terminal_rviz {

class RobotModelDisplay : public Display {
public:
    explicit RobotModelDisplay(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "std_msgs/msg/String"; }

    std::vector<std::string> getEnabledTopics() const override { return {topic_}; }
    TopicConfig getTopicConfig(const std::string& /*topic*/) override { return config_; }
    void setTopicConfig(const std::string& /*topic*/, const TopicConfig& config) override { config_ = config; }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    std::mutex mtx_;
    RobotRenderer robot_renderer_;
    TopicConfig config_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_
