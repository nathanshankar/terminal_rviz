#include "terminal_rviz/displays/range_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>
#include <cmath>

namespace terminal_rviz {

RangeDisplay::RangeDisplay(rclcpp::Node::SharedPtr node)
    : Display("Range", node) {}

void RangeDisplay::onInitialize() {}

void RangeDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 4; // Default Yellow
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<sensor_msgs::msg::Range>(
            topic, 10, [this, topic](const sensor_msgs::msg::Range::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_msgs_[topic] = msg;
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool RangeDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

std::vector<std::string> RangeDisplay::getEnabledTopics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return enabled_topics_;
}

TopicConfig RangeDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void RangeDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void RangeDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        sensor_msgs::msg::Range::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_msgs_.count(topic)) msg = latest_msgs_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform t_to_w;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(t_msg.transform, t_to_w);
        } catch (...) { continue; }

        uint8_t r_c = 255, g_c = 255, b_c = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
        }

        float r = msg->range;
        float fov = msg->field_of_view;
        float radius = r * std::tan(fov / 2.0f);
        tf2::Vector3 origin = t_to_w.getOrigin();

        int steps = 8;
        std::vector<tf2::Vector3> pts;
        for (int i = 0; i < steps; ++i) {
            float angle = 2.0f * M_PI * i / steps;
            pts.push_back(t_to_w * tf2::Vector3(r, radius * std::cos(angle), radius * std::sin(angle)));
        }

        for (int i = 0; i < steps; ++i) {
            renderer.draw_line(origin.x(), origin.y(), origin.z(), pts[i].x(), pts[i].y(), pts[i].z(), r_c, g_c, b_c, cfg.alpha);
            renderer.draw_line(pts[i].x(), pts[i].y(), pts[i].z(), pts[(i+1)%steps].x(), pts[(i+1)%steps].y(), pts[(i+1)%steps].z(), r_c, g_c, b_c, cfg.alpha);
        }
    }
}

ftxui::Element RangeDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" Range Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

} // namespace terminal_rviz
