#ifndef TERMINAL_RVIZ_DISPLAYS_MAP_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_MAP_DISPLAY_HPP_

#include <mutex>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class MapDisplay : public Display {
public:
    explicit MapDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "nav_msgs/msg/OccupancyGrid"; }

private:
    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    std::mutex mtx_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_msg_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_MAP_DISPLAY_HPP_
