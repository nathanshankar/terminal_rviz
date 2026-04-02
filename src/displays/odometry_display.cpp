#include "terminal_rviz/displays/odometry_display.hpp"
#include "terminal_rviz/config_helper.hpp"
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

void OdometryDisplay::onInitialize() {}

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
    cfg.color_style = "Flat";
    cfg.color_index = 1; // Red
    cfg.history_length = 100;
    cfg.size = 0.1f;
    cfg.alpha = 1.0f;
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, 10, [this, topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                geometry_msgs::msg::PoseStamped ps;
                ps.header = msg->header;
                ps.pose = msg->pose.pose;
                
                auto& history = histories_[topic];
                history.push_back(ps);
                auto& cfg = configs_[topic];
                while ((int)history.size() > cfg.history_length) history.pop_front();
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool OdometryDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig OdometryDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void OdometryDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
    auto& history = histories_[topic];
    while ((int)history.size() > config.history_length) history.pop_front();
}

void OdometryDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
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

        uint8_t r_c = 255, g_c = 255, b_c = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
        }

        for (const auto& ps : history) {
            tf2::Transform o_to_w;
            try {
                auto t_msg = tf_buffer->lookupTransform(fixed_frame, ps.header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(t_msg.transform, o_to_w);
            } catch (...) { continue; }

            tf2::Transform v_to_o;
            tf2::fromMsg(ps.pose, v_to_o);
            tf2::Transform v_to_w = o_to_w * v_to_o;

            float z_off = 0.05f;
            tf2::Vector3 base = v_to_w * tf2::Vector3(0, 0, 0);
            tf2::Vector3 tip = v_to_w * tf2::Vector3(cfg.size * 5.0f, 0, 0); 
            
            renderer.draw_arrow(base.x(), base.y(), base.z() + z_off, tip.x(), tip.y(), tip.z() + z_off, r_c, g_c, b_c, cfg.alpha, cfg.size * 2.0f);
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
            topics_ui.push_back(ConfigHelper::render_summary(topic, configs_[topic]));
        }
        count++;
    }
    return vbox({
        hbox({ text(" Odometry Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool OdometryDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
