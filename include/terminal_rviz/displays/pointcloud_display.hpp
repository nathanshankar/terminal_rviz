#ifndef TERMINAL_RVIZ_DISPLAYS_POINTCLOUD_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_POINTCLOUD_DISPLAY_HPP_

#include <mutex>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class PointCloudDisplay : public Display {
public:
    explicit PointCloudDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "sensor_msgs/msg/PointCloud2"; }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::mutex mtx_;
    sensor_msgs::msg::PointCloud2::SharedPtr current_msg_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_POINTCLOUD_DISPLAY_HPP_
