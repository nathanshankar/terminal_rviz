#ifndef TERMINAL_RVIZ_DISPLAYS_ODOMETRY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_ODOMETRY_HPP_

#include <mutex>
#include <deque>

#include "nav_msgs/msg/odometry.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class OdometryDisplay : public Display {
public:
    explicit OdometryDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "nav_msgs/msg/Odometry"; }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::mutex mtx_;
    std::deque<geometry_msgs::msg::Point> path_;
    size_t max_path_size_ = 100;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_ODOMETRY_HPP_
