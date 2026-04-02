#include "terminal_rviz/displays/pose_with_covariance_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

PoseWithCovarianceDisplay::PoseWithCovarianceDisplay(rclcpp::Node::SharedPtr node)
    : Display("PoseWithCovariance", node) {}

void PoseWithCovarianceDisplay::onInitialize() {}

void PoseWithCovarianceDisplay::setTopic(const std::string& topic) {
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
    cfg.r = 255; cfg.g = 255; cfg.b = 0; // Yellow default
    cfg.size = 0.5f;
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic, 10, [this, topic](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool PoseWithCovarianceDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void PoseWithCovarianceDisplay::callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_msgs_[topic] = msg;
}

TopicConfig PoseWithCovarianceDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void PoseWithCovarianceDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void PoseWithCovarianceDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg;
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

        tf2::Transform pose_in_frame;
        tf2::fromMsg(msg->pose.pose, pose_in_frame);
        tf2::Transform pose_in_world = frame_to_world * pose_in_frame;

        uint8_t r_c = 255, g_c = 255, b_c = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
        } else if (cfg.color_style == "RGB") {
            r_c = cfg.r; g_c = cfg.g; b_c = cfg.b;
        }

        tf2::Vector3 origin = pose_in_world * tf2::Vector3(0, 0, 0);
        tf2::Vector3 tip = pose_in_world * tf2::Vector3(cfg.size, 0, 0);

        renderer.draw_line(origin.x(), origin.y(), origin.z(), tip.x(), tip.y(), tip.z(), r_c, g_c, b_c, cfg.alpha);
        
        tf2::Vector3 dir = (tip - origin);
        if (dir.length() > 0.001) {
            dir.normalize();
            tf2::Vector3 side = pose_in_world.getBasis() * tf2::Vector3(0, 1, 0);
            float head_size = 0.2f * cfg.size;
            tf2::Vector3 p1 = tip - dir * head_size + side * head_size * 0.5f;
            tf2::Vector3 p2 = tip - dir * head_size - side * head_size * 0.5f;
            renderer.draw_line(tip.x(), tip.y(), tip.z(), p1.x(), p1.y(), p1.z(), r_c, g_c, b_c, cfg.alpha);
            renderer.draw_line(tip.x(), tip.y(), tip.z(), p2.x(), p2.y(), p2.z(), r_c, g_c, b_c, cfg.alpha);
        }
    }
}

ftxui::Element PoseWithCovarianceDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
    if (topics_ui.empty()) return text(enabled_topics_.empty() ? " No PoseWithCov topics active" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" PoseWithCov Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool PoseWithCovarianceDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
