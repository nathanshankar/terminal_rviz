#ifndef TERMINAL_RVIZ_DISPLAYS_POSE_HPP_
#define TERMINAL_RVIZ_DISPLAYS_POSE_HPP_

#include <mutex>
#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class PoseDisplay : public Display {
public:
    explicit PoseDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override { std::lock_guard<std::mutex> lock(mtx_); return enabled_topics_; }
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override { return "geometry_msgs/msg/PoseStamped"; }

private:
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& topic);

    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subs_;
    std::map<std::string, geometry_msgs::msg::PoseStamped::SharedPtr> latest_msgs_;
    std::map<std::string, TopicConfig> configs_;
    std::vector<std::string> enabled_topics_;
    
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_POSE_HPP_
