#ifndef TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_

#include <mutex>

#include "sensor_msgs/msg/image.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class ImageDisplay : public Display {
public:
    explicit ImageDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "sensor_msgs/msg/Image"; }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::mutex mtx_;
    sensor_msgs::msg::Image::SharedPtr current_msg_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_
