#ifndef TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_

#include <mutex>
#include <map>
#include <vector>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class ImageDisplay : public Display {
public:
    explicit ImageDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d() override;
    
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const;
    size_t getEnabledTopicCount() const { std::lock_guard<std::mutex> lock(mtx_); return enabled_topics_.size(); }
    std::string getMessageType() const override { return "sensor_msgs/msg/Image"; }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic);

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
    std::map<std::string, sensor_msgs::msg::Image::SharedPtr> latest_images_;
    std::vector<std::string> enabled_topics_;
    
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_IMAGE_DISPLAY_HPP_
