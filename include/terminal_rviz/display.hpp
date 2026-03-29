#ifndef TERMINAL_RVIZ_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "terminal_rviz/renderer.hpp"

namespace terminal_rviz {

class Display {
public:
    explicit Display(const std::string& name, rclcpp::Node::SharedPtr node);
    virtual ~Display() = default;

    virtual void onInitialize() {}
    virtual void update(double dt) {}
    virtual void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) = 0;

    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    void toggle() { enabled_ = !enabled_; }
    const std::string& getName() const { return name_; }
    
    virtual std::string getTopic() const { return topic_; }
    virtual void setTopic(const std::string& topic) { topic_ = topic; }
    virtual std::string getMessageType() const = 0;

protected:
    std::string name_;
    std::string topic_;
    rclcpp::Node::SharedPtr node_;
    bool enabled_ = true;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAY_HPP_
