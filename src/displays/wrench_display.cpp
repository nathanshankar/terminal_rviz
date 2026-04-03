#include "terminal_rviz/displays/wrench_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

WrenchDisplay::WrenchDisplay(rclcpp::Node::SharedPtr node)
    : Display("Wrench", node) {}

void WrenchDisplay::onInitialize() {}

void WrenchDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 1;   // Red for Linear
    cfg.color_index_2 = 5; // Cyan for Angular
    cfg.size = 0.1f;
    configs_[topic] = cfg;

    
    try {
        subs_[topic] = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            topic, 10, [this, topic](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_msgs_[topic] = msg;
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool WrenchDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig WrenchDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void WrenchDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void WrenchDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        geometry_msgs::msg::WrenchStamped::SharedPtr msg;
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

        uint8_t r_f = 255, g_f = 255, b_f = 255;
        uint8_t r_t = 255, g_t = 255, b_t = 255;
        
        static const uint8_t pr[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
        static const uint8_t pg[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
        static const uint8_t pb[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};

        if (cfg.color_style == "Flat") {
            int i1 = cfg.color_index % 10;
            r_f = pr[i1]; g_f = pg[i1]; b_f = pb[i1];
            
            int i2 = cfg.color_index_2 % 10;
            r_t = pr[i2]; g_t = pg[i2]; b_t = pb[i2];
        }

        tf2::Vector3 origin = frame_to_world * tf2::Vector3(0, 0, 0);
        
        // Linear Force Arrow
        tf2::Vector3 force_vec(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        tf2::Vector3 tip_f = frame_to_world * (force_vec * cfg.size);
        renderer.draw_arrow(origin.x(), origin.y(), origin.z(), tip_f.x(), tip_f.y(), tip_f.z(), r_f, g_f, b_f, cfg.alpha, 0.5f);

        // Angular Torque: Arrow + Circle
        tf2::Vector3 torque_vec(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
        float t_len = torque_vec.length();
        if (t_len > 1e-6) {
            tf2::Vector3 torque_dir = torque_vec.normalized();
            tf2::Vector3 tip_t = frame_to_world * (torque_vec * cfg.size);
            
            // Torque Arrow
            renderer.draw_arrow(origin.x(), origin.y(), origin.z(), tip_t.x(), tip_t.y(), tip_t.z(), r_t, g_t, b_t, cfg.alpha, 0.5f);
            
            // Torque Circle
            float radius = 0.2f * cfg.size * t_len;
            const int segments = 16;
            tf2::Vector3 v1, v2;
            if (std::abs(torque_dir.z()) < 0.9f) v1 = tf2::Vector3(0,0,1).cross(torque_dir).normalized();
            else v1 = tf2::Vector3(0,1,0).cross(torque_dir).normalized();
            v2 = torque_dir.cross(v1).normalized();

            for (int i = 0; i < segments; ++i) {
                float a1 = 2.0f * M_PI * i / segments;
                float a2 = 2.0f * M_PI * (i + 1) / segments;
                tf2::Vector3 p1_local = v1 * std::cos(a1) * radius + v2 * std::sin(a1) * radius;
                tf2::Vector3 p2_local = v1 * std::cos(a2) * radius + v2 * std::sin(a2) * radius;
                tf2::Vector3 p1 = frame_to_world * p1_local;
                tf2::Vector3 p2 = frame_to_world * p2_local;
                renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), r_t, g_t, b_t, cfg.alpha);
            }
        }
    }
}

ftxui::Element WrenchDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" Wrench Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}



bool WrenchDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
