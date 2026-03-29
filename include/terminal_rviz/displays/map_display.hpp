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
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "nav_msgs/msg/OccupancyGrid"; }

    float getCenterX() const { return map_center_x_; }
    float getCenterY() const { return map_center_y_; }
    float getWidth() const { return map_width_m_; }
    float getHeight() const { return map_height_m_; }

    private:
    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    std::mutex mtx_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    float map_center_x_ = 0.0f;
    float map_center_y_ = 0.0f;
    float map_width_m_ = 10.0f;
    float map_height_m_ = 10.0f;
    };

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_MAP_DISPLAY_HPP_
