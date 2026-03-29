#include "terminal_rviz/displays/pointcloud_display.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace terminal_rviz {

PointCloudDisplay::PointCloudDisplay(rclcpp::Node::SharedPtr node)
    : Display("PointCloud", node) {}

void PointCloudDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "points");
    setTopic(topic_);
}

void PointCloudDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_, 10, std::bind(&PointCloudDisplay::callback, this, std::placeholders::_1));
}

void PointCloudDisplay::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_msg_ = msg;
}

void PointCloudDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    // Sample points if too many (for performance)
    size_t total_points = msg->width * msg->height;
    size_t skip = std::max((size_t)1, total_points / 5000);

    for (size_t i = 0; i < total_points; i += skip, iter_x += skip, iter_y += skip, iter_z += skip) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        // Simple color based on Z
        uint8_t r = static_cast<uint8_t>(std::clamp(128.0f + z * 50.0f, 0.0f, 255.0f));
        uint8_t g = static_cast<uint8_t>(std::clamp(128.0f - z * 50.0f, 0.0f, 255.0f));
        uint8_t b = 200;
        
        renderer.draw_point(x, y, z, ftxui::Color::RGB(r, g, b), canvas);
    }
}

} // namespace terminal_rviz
