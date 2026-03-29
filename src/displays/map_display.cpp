#include "terminal_rviz/displays/map_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

void MapDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    nav_msgs::msg::OccupancyGrid::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    tf2::Transform map_to_world;
    try {
        auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(transform_msg.transform, map_to_world);
    } catch (...) { return; }

    float res = msg->info.resolution;
    float ox = msg->info.origin.position.x;
    float oy = msg->info.origin.position.y;
    int width = msg->info.width;
    int height = msg->info.height;

    int skip = std::max(1, (width * height) / 10000);

    for (int i = 0; i < width * height; i += skip) {
        int8_t val = msg->data[i];
        if (val > 50) { 
            int xi = i % width;
            int yi = i / width;
            tf2::Vector3 p_local(ox + xi * res, oy + yi * res, -0.01f);
            tf2::Vector3 p_world = map_to_world * p_local;
            renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), ftxui::Color::GrayLight);
        }
    }
}

} // namespace terminal_rviz
