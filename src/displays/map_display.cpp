#include "terminal_rviz/displays/map_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace terminal_rviz {

MapDisplay::MapDisplay(rclcpp::Node::SharedPtr node)
    : Display("Map", node) {}

void MapDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "map");
    setTopic(topic_);
}

void MapDisplay::setTopic(const std::string& topic) {
    try {
        sub_.reset();
        topic_ = topic;
        sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_, rclcpp::QoS(1).transient_local(), std::bind(&MapDisplay::callback, this, std::placeholders::_1));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Map: Failed to subscribe to %s: %s", topic.c_str(), e.what());
    }
}

void MapDisplay::callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_map_ = msg;

    // Calculate Map Center in its own frame
    map_width_m_ = msg->info.width * msg->info.resolution;
    map_height_m_ = msg->info.height * msg->info.resolution;
    map_center_x_ = msg->info.origin.position.x + (map_width_m_ / 2.0f);
    map_center_y_ = msg->info.origin.position.y + (map_height_m_ / 2.0f);
}

void MapDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    nav_msgs::msg::OccupancyGrid::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = latest_map_;
    }

    if (!msg) return;

    tf2::Transform map_to_world;
    try {
        auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
        map_to_world.setOrigin(tf2::Vector3(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z));
        map_to_world.setRotation(tf2::Quaternion(transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w));
    } catch (...) { return; }

    float res = msg->info.resolution;
    float ox = msg->info.origin.position.x;
    float oy = msg->info.origin.position.y;
    uint32_t width = msg->info.width;
    uint32_t height = msg->info.height;

    // Draw Map Border
    tf2::Vector3 bl = map_to_world * tf2::Vector3(ox, oy, 0);
    tf2::Vector3 br = map_to_world * tf2::Vector3(ox + width * res, oy, 0);
    tf2::Vector3 tr = map_to_world * tf2::Vector3(ox + width * res, oy + height * res, 0);
    tf2::Vector3 tl = map_to_world * tf2::Vector3(ox, oy + height * res, 0);
    renderer.draw_line(bl.x(), bl.y(), bl.z(), br.x(), br.y(), br.z(), ftxui::Color::Yellow);
    renderer.draw_line(br.x(), br.y(), br.z(), tr.x(), tr.y(), tr.z(), ftxui::Color::Yellow);
    renderer.draw_line(tr.x(), tr.y(), tr.z(), tl.x(), tl.y(), tl.z(), ftxui::Color::Yellow);
    renderer.draw_line(tl.x(), tl.y(), tl.z(), bl.x(), bl.y(), bl.z(), ftxui::Color::Yellow);

    // Dynamic subsampling - INCREASED DENSITY
    uint32_t total = width * height;
    uint32_t max_pts = 50000; 
    uint32_t skip = std::max(1U, total / max_pts);

    for (uint32_t i = 0; i < total; i += skip) {
        int8_t val = msg->data[i];
        if (val == -1) continue; // Unknown

        uint32_t xi = i % width;
        uint32_t yi = i / width;
        tf2::Vector3 p_local(ox + xi * res, oy + yi * res, -0.02f); // Slightly below ground
        tf2::Vector3 p_world = map_to_world * p_local;

        ftxui::Color col;
        if (val > 50) col = ftxui::Color::Magenta; // Occupied (High Contrast)
        else col = ftxui::Color::White;            // Known / Free space

        renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), col);
    }
}

} // namespace terminal_rviz
