#include "terminal_rviz/displays/pose_array_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

PoseArrayDisplay::PoseArrayDisplay(rclcpp::Node::SharedPtr node)
    : Display("PoseArray", node) {}

void PoseArrayDisplay::onInitialize() {}

void PoseArrayDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 3; // Blue
    cfg.size = 0.2f;
    configs_[topic] = cfg;
    
    subs_[topic] = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        topic, 10, [this, topic](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_msgs_[topic] = msg;
        });
}

bool PoseArrayDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig PoseArrayDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void PoseArrayDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void PoseArrayDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        geometry_msgs::msg::PoseArray::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_msgs_.count(topic)) msg = latest_msgs_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform frame_to_world;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(t_msg.transform, frame_to_world);
        } catch (...) { continue; }

        uint8_t r_c = 255, g_c = 255, b_c = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
        }
        
        for (const auto& pose : msg->poses) {
            tf2::Transform p_tf;
            tf2::fromMsg(pose, p_tf);
            tf2::Transform p_world = frame_to_world * p_tf;
            tf2::Vector3 origin = p_world.getOrigin();
            tf2::Vector3 tip = p_world * tf2::Vector3(cfg.size * 2.0f, 0, 0);
            renderer.draw_arrow(origin.x(), origin.y(), origin.z(), tip.x(), tip.y(), tip.z(), r_c, g_c, b_c, cfg.alpha, cfg.size);
        }
    }
}

ftxui::Element PoseArrayDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" PoseArray Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}


bool PoseArrayDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
