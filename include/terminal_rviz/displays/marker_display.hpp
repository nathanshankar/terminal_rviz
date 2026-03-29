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
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return preferred_type_; }
    void setPreferredType(const std::string& type) { preferred_type_ = type; }

private:
    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
    
    std::mutex mtx_;
    std::map<std::string, visualization_msgs::msg::Marker> markers_;
    std::string preferred_type_ = "visualization_msgs/msg/Marker";
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_MARKER_DISPLAY_HPP_
