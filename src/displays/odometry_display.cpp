#include "terminal_rviz/displays/odometry_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <cmath>
#include <algorithm>

namespace terminal_rviz {

OdometryDisplay::OdometryDisplay(rclcpp::Node::SharedPtr node)
    : Display("Odometry", node) {}

void OdometryDisplay::onInitialize() {
}

void OdometryDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        subs_.erase(topic);
        histories_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg; 
    // Defaults for Odometry
    if (enabled_topics_.size() == 1) { cfg.r = 255; cfg.g = 0; cfg.b = 0; } // Red
    else { cfg.r = 0; cfg.g = 255; cfg.b = 0; } // Green
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, 10, [this, topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool OdometryDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void OdometryDisplay::callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = msg->pose.pose;
    
    auto& history = histories_[topic];
    history.push_back(ps);
    if (history.size() > max_history_) history.pop_front();
}

TopicConfig OdometryDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void OdometryDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void OdometryDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        std::deque<geometry_msgs::msg::PoseStamped> history;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (histories_.count(topic)) history = histories_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        auto col = ftxui::Color::RGB(cfg.r, cfg.g, cfg.b);

        for (const auto& ps : history) {
            tf2::Transform o_to_w;
            try {
                auto t_msg = tf_buffer->lookupTransform(fixed_frame, ps.header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(t_msg.transform, o_to_w);
            } catch (...) { continue; }

            tf2::Transform v_to_o;
            tf2::fromMsg(ps.pose, v_to_o);
            tf2::Transform v_to_w = o_to_w * v_to_o;

            float z_off = 0.1f;
            tf2::Vector3 base = v_to_w * tf2::Vector3(0, 0, 0);
            tf2::Vector3 tip = v_to_w * tf2::Vector3(cfg.size * 12.0f, 0, 0); 
            
            renderer.draw_line(base.x(), base.y(), base.z() + z_off, tip.x(), tip.y(), tip.z() + z_off, col);
            
            tf2::Vector3 dir = (tip - base).normalized();
            tf2::Vector3 side = v_to_w.getBasis() * tf2::Vector3(0, 1, 0);
            tf2::Vector3 left_wing = tip - dir * 0.2 + side * 0.15;
            tf2::Vector3 right_wing = tip - dir * 0.2 - side * 0.15;
            
            renderer.draw_line(tip.x(), tip.y(), tip.z() + z_off, left_wing.x(), left_wing.y(), left_wing.z() + z_off, col);
            renderer.draw_line(tip.x(), tip.y(), tip.z() + z_off, right_wing.x(), right_wing.y(), right_wing.z() + z_off, col);
        }
    }
}

ftxui::Element OdometryDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 4) { 
            auto& cfg = configs_[topic];
            topics_ui.push_back(vbox({
                hbox({ text(" Topic: ") | bold, text(topic) | color(Color::RGB(cfg.r, cfg.g, cfg.b)) }),
                hbox({
                    text(" Size:  "),
                    text(std::to_string(cfg.size).substr(0,4)) | color(Color::Cyan),
                    filler(),
                    text(" R/G/B: "),
                    text(std::to_string(cfg.r) + "/" + std::to_string(cfg.g) + "/" + std::to_string(cfg.b)) | dim,
                }),
                separator(),
            }));
        }
        count++;
    }
    
    if (topics_ui.empty()) {
        if (enabled_topics_.empty()) return text(" No Odometry topics active") | dim | center;
        return text(" (End of list)") | dim | center;
    }

    return vbox({
        hbox({ text(" Odometry Settings ") | bold | color(Color::Yellow), filler(), text(" [Scroll/Cycle] ") | dim }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool OdometryDisplay::handle_event(ftxui::Event event, int scroll_offset) {
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
