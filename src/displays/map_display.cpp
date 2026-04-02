#include "terminal_rviz/displays/map_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

MapDisplay::MapDisplay(rclcpp::Node::SharedPtr node)
    : Display("Map", node) {}

void MapDisplay::onInitialize() {}

void MapDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        subs_.erase(topic);
        latest_maps_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg;
    cfg.style = "Map";
    cfg.alpha = 0.7f;
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic, rclcpp::QoS(1).transient_local(), [this, topic](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_maps_[topic] = msg;
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool MapDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig MapDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void MapDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

float MapDisplay::getCenterX() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (latest_maps_.empty()) return 0.0f;
    auto msg = latest_maps_.begin()->second;
    return msg->info.origin.position.x + (msg->info.width * msg->info.resolution / 2.0f);
}

float MapDisplay::getCenterY() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (latest_maps_.empty()) return 0.0f;
    auto msg = latest_maps_.begin()->second;
    return msg->info.origin.position.y + (msg->info.height * msg->info.resolution / 2.0f);
}

float MapDisplay::getWidth() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (latest_maps_.empty()) return 10.0f;
    return latest_maps_.begin()->second->info.width * latest_maps_.begin()->second->info.resolution;
}

float MapDisplay::getHeight() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (latest_maps_.empty()) return 10.0f;
    return latest_maps_.begin()->second->info.height * latest_maps_.begin()->second->info.resolution;
}

void MapDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        nav_msgs::msg::OccupancyGrid::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_maps_.count(topic)) msg = latest_maps_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform map_to_world;
        try {
            auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(transform_msg.transform, map_to_world);
        } catch (...) { continue; }

        float res = msg->info.resolution;
        float ox = msg->info.origin.position.x;
        float oy = msg->info.origin.position.y;
        uint32_t width = msg->info.width;
        uint32_t height = msg->info.height;

        // Draw Map Border
        auto border_col = ftxui::Color::Yellow;
        tf2::Vector3 bl = map_to_world * tf2::Vector3(ox, oy, 0);
        tf2::Vector3 br = map_to_world * tf2::Vector3(ox + width * res, oy, 0);
        tf2::Vector3 tr = map_to_world * tf2::Vector3(ox + width * res, oy + height * res, 0);
        tf2::Vector3 tl = map_to_world * tf2::Vector3(ox, oy + height * res, 0);
        renderer.draw_line(bl.x(), bl.y(), bl.z(), br.x(), br.y(), br.z(), border_col, cfg.alpha);
        renderer.draw_line(br.x(), br.y(), br.z(), tr.x(), tr.y(), tr.z(), border_col, cfg.alpha);
        renderer.draw_line(tr.x(), tr.y(), tr.z(), tl.x(), tl.y(), tl.z(), border_col, cfg.alpha);
        renderer.draw_line(tl.x(), tl.y(), tl.z(), bl.x(), bl.y(), bl.z(), border_col, cfg.alpha);

        uint32_t total = width * height;
        uint32_t max_pts = 10000; 
        uint32_t skip = std::max(1U, total / max_pts);

        for (uint32_t i = 0; i < total; i += skip) {
            int8_t val = msg->data[i];
            if (val == -1) continue;

            uint32_t xi = i % width;
            uint32_t yi = i / width;
            tf2::Vector3 p_local(ox + xi * res, oy + yi * res, -0.01f); 
            tf2::Vector3 p_world = map_to_world * p_local;

            uint8_t r_c, g_c, b_c;
            if (cfg.style == "Costmap") {
                if (val > 90) { r_c = 255; g_c = 255; b_c = 255; }
                else if (val > 50) { r_c = 128; g_c = 128; b_c = 128; }
                else { r_c = 0; g_c = 0; b_c = 0; }
            } else { // "Map" style
                if (val > 50) { r_c = 255; g_c = 0; b_c = 255; } // Magenta
                else { r_c = 255; g_c = 255; b_c = 255; } // White
            }

            renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), r_c, g_c, b_c, cfg.alpha);
        }
    }
}

ftxui::Element MapDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 4) { 
            topics_ui.push_back(ConfigHelper::render_summary(topic, configs_[topic]));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_topics_.empty() ? " No Map topics active" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" Map Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool MapDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
