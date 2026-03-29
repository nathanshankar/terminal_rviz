#include "terminal_rviz/displays/map_display.hpp"

namespace terminal_rviz {

MapDisplay::MapDisplay(rclcpp::Node::SharedPtr node)
    : Display("Map", node) {}

void MapDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "map");
    setTopic(topic_);
}

void MapDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_, 10, std::bind(&MapDisplay::callback, this, std::placeholders::_1));
}

void MapDisplay::callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_msg_ = msg;
}

void MapDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    nav_msgs::msg::OccupancyGrid::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    float res = msg->info.resolution;
    float ox = msg->info.origin.position.x;
    float oy = msg->info.origin.position.y;
    int width = msg->info.width;
    int height = msg->info.height;

    // Subsample for performance
    int skip = std::max(1, (width * height) / 10000);

    for (int i = 0; i < width * height; i += skip) {
        int8_t val = msg->data[i];
        if (val > 50) { // Occupied
            int xi = i % width;
            int yi = i / width;
            float x = ox + xi * res;
            float y = oy + yi * res;
            renderer.draw_point(x, y, -0.01f, ftxui::Color::GrayLight, canvas);
        }
    }
}

} // namespace terminal_rviz
