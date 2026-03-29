#ifndef TERMINAL_RVIZ_DISPLAYS_LASERSCAN_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_LASERSCAN_DISPLAY_HPP_

#include <mutex>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class LaserScanDisplay : public Display {
public:
    explicit LaserScanDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "sensor_msgs/msg/LaserScan"; }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    std::mutex mtx_;
    sensor_msgs::msg::LaserScan::SharedPtr current_msg_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_LASERSCAN_DISPLAY_HPP_
