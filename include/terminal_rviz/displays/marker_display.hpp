#ifndef TERMINAL_RVIZ_DISPLAYS_MARKER_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_MARKER_DISPLAY_HPP_

#include <mutex>
#include <map>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class MarkerDisplay : public Display {
public:
    explicit MarkerDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override { std::lock_guard<std::mutex> lock(mtx_); return enabled_topics_; }
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override { return preferred_type_; }
    void setPreferredType(const std::string& type) { preferred_type_ = type; }

private:
    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg, const std::string& topic);
    void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, const std::string& topic);

    std::map<std::string, rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr> marker_subs_;
    std::map<std::string, rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_array_subs_;

    mutable std::mutex mtx_;
    std::map<std::string, std::map<std::string, visualization_msgs::msg::Marker>> marker_store_;
    std::map<std::string, TopicConfig> configs_;
    std::vector<std::string> enabled_topics_;
    std::string preferred_type_ = "visualization_msgs/msg/Marker";
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_MARKER_DISPLAY_HPP_
