#ifndef TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_

#include <mutex>
#include <map>
#include <string>
#include <set>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class TFDisplay : public Display {
public:
    explicit TFDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    std::string getMessageType() const override { return "TF"; }

    std::vector<std::string> getDiscoveredFrames();
    void toggleFrame(const std::string& frame);
    bool isFrameEnabled(const std::string& frame) const;

    bool isTopicEnabled(const std::string& topic) const override { return isFrameEnabled(topic); }
    std::vector<std::string> getEnabledTopics() const override;
    void setTopic(const std::string& topic) override { toggleFrame(topic); }
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset) override;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::string> enabled_frames_list_; // Order preservation for sidebar
    std::map<std::string, TopicConfig> configs_;
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_
