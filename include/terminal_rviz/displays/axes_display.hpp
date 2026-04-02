#ifndef TERMINAL_RVIZ_DISPLAYS_AXES_HPP_
#define TERMINAL_RVIZ_DISPLAYS_AXES_HPP_

#include <mutex>
#include <set>
#include <map>
#include <string>
#include <vector>

#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class AxesDisplay : public Display {
public:
    explicit AxesDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override { std::lock_guard<std::mutex> lock(mtx_); return enabled_frames_; }
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override;

    // Helpers for frame management
    void toggleFrame(const std::string& frame);
    bool isFrameEnabled(const std::string& frame) const;

private:
    std::vector<std::string> enabled_frames_;
    std::map<std::string, TopicConfig> configs_;
    
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_AXES_HPP_
