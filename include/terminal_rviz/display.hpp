#ifndef TERMINAL_RVIZ_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "terminal_rviz/renderer.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"

namespace terminal_rviz {

struct TopicConfig {
    std::string color_style = "Flat"; // Flat, Axis, Intensity
    float alpha = 1.0f;
    float size = 0.05f;
    uint8_t r = 255, g = 255, b = 255;
    std::string axis = "Z"; // X, Y, Z
};

class Display {
public:
    explicit Display(const std::string& name, rclcpp::Node::SharedPtr node);
    virtual ~Display() = default;

    virtual void onInitialize() {}
    virtual void update(double dt) {}
    virtual void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;
    virtual ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) { return ftxui::filler(); }
    virtual bool handle_event(ftxui::Event event, int scroll_offset = 0) { return false; }

    void setName(const std::string& name) { name_ = name; }
    bool isEnabled() const { return enabled_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    void toggle() { enabled_ = !enabled_; }
    
    bool isAdded() const { return added_; }
    void setAdded(bool added) { added_ = added; }
    
    virtual bool isTopicEnabled(const std::string& /*topic*/) const { return false; }
    virtual std::vector<std::string> getEnabledTopics() const { return {}; }

    virtual TopicConfig getTopicConfig(const std::string& /*topic*/) { return TopicConfig(); }
    virtual void setTopicConfig(const std::string& /*topic*/, const TopicConfig& /*config*/) {}
    
    std::string getName() const { return name_; }
    
    virtual std::string getTopic() const { return topic_; }
    virtual void setTopic(const std::string& topic) { topic_ = topic; }
    virtual std::string getMessageType() const = 0;

protected:
    std::string name_;
    std::string topic_;
    rclcpp::Node::SharedPtr node_;
    bool enabled_ = false;
    bool added_ = false;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAY_HPP_
