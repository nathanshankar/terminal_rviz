#include "terminal_rviz/displays/map_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

MapDisplay::MapDisplay(rclcpp::Node::SharedPtr node)
    : Display("Map", node) {}

void MapDisplay::onInitialize() {
}

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
    configs_[topic] = TopicConfig();
    
    try {
        subs_[topic] = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic, rclcpp::QoS(1).transient_local(), [this, topic](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool MapDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void MapDisplay::callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_maps_[topic] = msg;
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

void MapDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
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
        renderer.draw_line(bl.x(), bl.y(), bl.z(), br.x(), br.y(), br.z(), border_col);
        renderer.draw_line(br.x(), br.y(), br.z(), tr.x(), tr.y(), tr.z(), border_col);
        renderer.draw_line(tr.x(), tr.y(), tr.z(), tl.x(), tl.y(), tl.z(), border_col);
        renderer.draw_line(tl.x(), tl.y(), tl.z(), bl.x(), bl.y(), bl.z(), border_col);

        uint32_t total = width * height;
        uint32_t max_pts = 20000; 
        uint32_t skip = std::max(1U, total / max_pts);

        for (uint32_t i = 0; i < total; i += skip) {
            int8_t val = msg->data[i];
            if (val == -1) continue;

            uint32_t xi = i % width;
            uint32_t yi = i / width;
            tf2::Vector3 p_local(ox + xi * res, oy + yi * res, -0.02f); 
            tf2::Vector3 p_world = map_to_world * p_local;

            ftxui::Color col;
            if (val > 50) col = ftxui::Color::Magenta; 
            else col = ftxui::Color::White; 

            renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), col);
        }
    }
}

ftxui::Element MapDisplay::render_2d(bool /*nav2_active*/, int /*config_scroll*/) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    
    Elements topics_ui;
    for (const auto& topic : enabled_topics_) {
        topics_ui.push_back(vbox({
            hbox({ text(" Topic: ") | bold, text(topic) | color(Color::Yellow) }),
            separator(),
        }));
    }
    
    if (topics_ui.empty()) return text(" No Map topics active") | dim | center;

    return vbox({
        hbox({ text(" Map Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool MapDisplay::handle_event(ftxui::Event event, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
