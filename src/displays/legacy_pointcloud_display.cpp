#include "terminal_rviz/displays/legacy_pointcloud_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>
#include <limits>

namespace terminal_rviz {

LegacyPointCloudDisplay::LegacyPointCloudDisplay(rclcpp::Node::SharedPtr node)
    : Display("PointCloud", node) {}

void LegacyPointCloudDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        subs_.erase(topic);
        latest_msgs_.erase(topic);
        configs_.erase(topic);
        return;
    }
    enabled_topics_.push_back(topic);
    TopicConfig cfg;
    cfg.size = 0.0f;
    cfg.alpha = 1.0f;
    configs_[topic] = cfg;
    subs_[topic] = node_->create_subscription<sensor_msgs::msg::PointCloud>(
        topic, 10, [this, topic](const sensor_msgs::msg::PointCloud::SharedPtr msg) {
            callback(msg, topic);
        });
}

bool LegacyPointCloudDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void LegacyPointCloudDisplay::callback(const sensor_msgs::msg::PointCloud::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_msgs_[topic] = msg;
}

TopicConfig LegacyPointCloudDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void LegacyPointCloudDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void LegacyPointCloudDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& topic : enabled_topics_) {
        if (!latest_msgs_.count(topic)) continue;
        auto msg = latest_msgs_[topic];
        auto cfg = configs_[topic];

        tf2::Transform pc_to_world;
        try {
            auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(transform_msg.transform, pc_to_world);
        } catch (...) { continue; }

        size_t total_points = msg->points.size();
        if (total_points == 0) continue;

        size_t max_render = 10000;
        size_t skip = std::max((size_t)1, total_points / max_render);

        for (size_t i = 0; i < total_points; i += skip) {
            const auto& p = msg->points[i];
            tf2::Vector3 p_world = pc_to_world * tf2::Vector3(p.x, p.y, p.z);
            int sx, sy; float sz;
            if (renderer.project(p_world.x(), p_world.y(), p_world.z(), sx, sy, sz)) {
                uint8_t r_c = 255, g_c = 255, b_c = 255;
                if (cfg.color_style == "Flat") {
                    static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
                    static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
                    static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
                    int idx = cfg.color_index % 10;
                    r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
                }
                
                int r = (int)(cfg.size * 10.0f);
                if (r <= 0) renderer.plot(sx, sy, sz, r_c, g_c, b_c, cfg.alpha);
                else {
                    for (int dy = -r; dy <= r; ++dy) {
                        for (int dx = -r; dx <= r; ++dx) {
                            renderer.plot(sx + dx, sy + dy, sz, r_c, g_c, b_c, cfg.alpha);
                        }
                    }
                }
            }
        }
    }
}

ftxui::Element LegacyPointCloudDisplay::render_2d(bool, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 5) {
            topics_ui.push_back(ConfigHelper::render_summary(topic, configs_[topic]));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_topics_.empty() ? " No topics active" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" LegacyPointCloud Topics ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool LegacyPointCloudDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;
    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12);
    if (ry < 0 || ry >= 10) return false;
    int item_idx = ry + scroll_offset;
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_topics_.size()) return true;
    return false;
}

} // namespace terminal_rviz
