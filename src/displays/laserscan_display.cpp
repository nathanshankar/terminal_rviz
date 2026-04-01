#include "terminal_rviz/displays/laserscan_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>
#include <cmath>

namespace terminal_rviz {

LaserScanDisplay::LaserScanDisplay(rclcpp::Node::SharedPtr node)
    : Display("LaserScan", node) {}

void LaserScanDisplay::onInitialize() {
}

void LaserScanDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        subs_.erase(topic);
        latest_scans_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg; cfg.r = 255; cfg.g = 0; cfg.b = 0; // Default Red for Laser
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            topic, 10, [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool LaserScanDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void LaserScanDisplay::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_scans_[topic] = msg;
}

TopicConfig LaserScanDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void LaserScanDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void LaserScanDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        sensor_msgs::msg::LaserScan::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_scans_.count(topic)) msg = latest_scans_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform scan_to_world;
        try {
            auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(transform_msg.transform, scan_to_world);
        } catch (...) { continue; }

        auto col = ftxui::Color::RGB(cfg.r, cfg.g, cfg.b);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max || std::isnan(range)) continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            float lx = range * std::cos(angle), ly = range * std::sin(angle);
            
            tf2::Vector3 p_world = scan_to_world * tf2::Vector3(lx, ly, 0);
            int sx, sy; float sz;
            if (renderer.project(p_world.x(), p_world.y(), p_world.z(), sx, sy, sz)) {
                renderer.plot(sx, sy, sz, col);
            }
        }
    }
}

ftxui::Element LaserScanDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 4) { 
            auto& cfg = configs_[topic];
            topics_ui.push_back(vbox({
                hbox({ text(" Topic: ") | bold, text(topic) | color(Color::Red) }),
                hbox({
                    text(" Color: "),
                    text(" [" + cfg.color_style + "] ") | color(Color::Cyan),
                    filler(),
                    text(" Alpha: "),
                    text(std::to_string(cfg.alpha).substr(0,4)) | color(Color::GrayLight),
                }),
                separator(),
            }));
        }
        count++;
    }
    
    if (topics_ui.empty()) {
        if (enabled_topics_.empty()) return text(" No LaserScan topics active") | dim | center;
        return text(" (End of list)") | dim | center;
    }

    return vbox({
        hbox({ text(" LaserScan Settings ") | bold | color(Color::Yellow), filler(), text(" [Scroll/Cycle] ") | dim }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool LaserScanDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;

    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12); 
    if (ry < 0 || ry >= 10) return false;

    int item_idx = (ry / 3) + scroll_offset;
    
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_topics_.size()) {
        std::string topic = enabled_topics_[item_idx];
        auto& cfg = configs_[topic];
        
        // Cycle colors
        if (cfg.r == 255 && cfg.g == 0) { cfg.r = 0; cfg.g = 255; }
        else if (cfg.g == 255) { cfg.g = 0; cfg.b = 255; }
        else { cfg.b = 0; cfg.r = 255; }
        return true;
    }
    return false;
}

} // namespace terminal_rviz
