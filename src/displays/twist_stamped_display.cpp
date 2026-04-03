#include "terminal_rviz/displays/twist_stamped_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

TwistStampedDisplay::TwistStampedDisplay(rclcpp::Node::SharedPtr node)
    : Display("TwistStamped", node) {}

void TwistStampedDisplay::onInitialize() {}

void TwistStampedDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 7; // Orange default
    cfg.size = 1.0f;
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            topic, 10, [this, topic](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_msgs_[topic] = msg;
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool TwistStampedDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig TwistStampedDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void TwistStampedDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void TwistStampedDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        geometry_msgs::msg::TwistStamped::SharedPtr msg;
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

        uint8_t r_lin = 255, g_lin = 255, b_lin = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_lin = preset_r[idx]; g_lin = preset_g[idx]; b_lin = preset_b[idx];
        } else if (cfg.color_style == "RGB") {
            r_lin = cfg.r; g_lin = cfg.g; b_lin = cfg.b;
        }

        tf2::Vector3 origin = frame_to_world * tf2::Vector3(0, 0, 0);
        
        // Linear velocity arrow
        tf2::Vector3 linear_vec(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        tf2::Vector3 tip_l = frame_to_world * (linear_vec * cfg.size);
        renderer.draw_arrow(origin.x(), origin.y(), origin.z(), tip_l.x(), tip_l.y(), tip_l.z(), r_lin, g_lin, b_lin, cfg.alpha, 0.5f);

        // Angular velocity arrow (Cyan: 0, 255, 255)
        tf2::Vector3 angular_vec(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
        tf2::Vector3 tip_a = frame_to_world * (angular_vec * cfg.size);
        renderer.draw_arrow(origin.x(), origin.y(), origin.z(), tip_a.x(), tip_a.y(), tip_a.z(), (uint8_t)0, (uint8_t)255, (uint8_t)255, cfg.alpha, 0.5f);
    }
}

ftxui::Element TwistStampedDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" Twist Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool TwistStampedDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
