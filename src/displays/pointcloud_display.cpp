#include "terminal_rviz/displays/pointcloud_display.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

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

void PointCloudDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    tf2::Transform pc_to_world;
    try {
        auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(transform_msg.transform, pc_to_world);
    } catch (...) { return; }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    size_t total_points = msg->width * msg->height;
    if (total_points == 0) return;

    float min_x = 0.0f, max_x = 10.0f; 
    size_t max_render_points = 10000;
    size_t skip = std::max((size_t)1, total_points / max_render_points);
    float x_range_inv = 1.0f / std::max(0.1f, max_x - min_x);

    for (size_t i = 0; i < total_points; i += skip, iter_x += skip, iter_y += skip, iter_z += skip) {
        tf2::Vector3 p_local(*iter_x, *iter_y, *iter_z);
        tf2::Vector3 p_world = pc_to_world * p_local;

        int sx, sy;
        float sz;
        if (renderer.project(p_world.x(), p_world.y(), p_world.z(), sx, sy, sz)) {
            int r = (1.5f / (sz + 0.1f)) > 1.5f ? 1 : 0;      

            float v = std::clamp(static_cast<float>((p_local.x() - min_x) * x_range_inv), 0.0f, 1.0f);
            uint8_t r_col = 0, g_col = 0, b_col = 0;
            if (v < 0.25f) { r_col = 255; g_col = static_cast<uint8_t>(v * 1020); }
            else if (v < 0.5f) { r_col = static_cast<uint8_t>((0.5f - v) * 1020); g_col = 255; }
            else if (v < 0.75f) { g_col = 255; b_col = static_cast<uint8_t>((v - 0.5f) * 1020); }
            else { g_col = static_cast<uint8_t>((1.0f - v) * 1020); b_col = 255; }
            ftxui::Color col = ftxui::Color::RGB(r_col, g_col, b_col);

            if (r == 0) {
                renderer.plot(sx, sy, sz, col, canvas);
            } else {
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dx = -r; dx <= r; ++dx) {
                        renderer.plot(sx + dx, sy + dy, sz, col, canvas);
                    }
                }
            }
        }
    }
}

} // namespace terminal_rviz
