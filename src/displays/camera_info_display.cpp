#include "terminal_rviz/displays/camera_info_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

CameraInfoDisplay::CameraInfoDisplay(rclcpp::Node::SharedPtr node)
    : Display("CameraInfo", node) {}

void CameraInfoDisplay::onInitialize() {}

void CameraInfoDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 5; // Default Cyan
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            topic, 10, [this, topic](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool CameraInfoDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

std::vector<std::string> CameraInfoDisplay::getEnabledTopics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return enabled_topics_;
}

void CameraInfoDisplay::callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_msgs_[topic] = msg;
}

TopicConfig CameraInfoDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void CameraInfoDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void CameraInfoDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        sensor_msgs::msg::CameraInfo::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_msgs_.count(topic)) msg = latest_msgs_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg || msg->k[0] == 0) continue;

        tf2::Transform cam_to_world;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(t_msg.transform, cam_to_world);
        } catch (...) { continue; }

        uint8_t r_c = 255, g_c = 255, b_c = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
        }

        float fx = msg->k[0];
        float fy = msg->k[4];
        float cx = msg->k[2];
        float cy = msg->k[5];
        float width = msg->width;
        float height = msg->height;

        float far_dist = 1.0f; // 1 meter frustum
        
        // Frustum corners in camera frame
        auto get_pt = [&](float x, float y) {
            float dx = (x - cx) * far_dist / fx;
            float dy = (y - cy) * far_dist / fy;
            return cam_to_world * tf2::Vector3(dx, dy, far_dist);
        };

        tf2::Vector3 origin = cam_to_world.getOrigin();
        tf2::Vector3 p1 = get_pt(0, 0);
        tf2::Vector3 p2 = get_pt(width, 0);
        tf2::Vector3 p3 = get_pt(width, height);
        tf2::Vector3 p4 = get_pt(0, height);

        // Lines from origin to corners
        renderer.draw_line(origin.x(), origin.y(), origin.z(), p1.x(), p1.y(), p1.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(origin.x(), origin.y(), origin.z(), p2.x(), p2.y(), p2.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(origin.x(), origin.y(), origin.z(), p3.x(), p3.y(), p3.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(origin.x(), origin.y(), origin.z(), p4.x(), p4.y(), p4.z(), r_c, g_c, b_c, cfg.alpha);

        // Image plane rectangle
        renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(p2.x(), p2.y(), p2.z(), p3.x(), p3.y(), p3.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(p3.x(), p3.y(), p3.z(), p4.x(), p4.y(), p4.z(), r_c, g_c, b_c, cfg.alpha);
        renderer.draw_line(p4.x(), p4.y(), p4.z(), p1.x(), p1.y(), p1.z(), r_c, g_c, b_c, cfg.alpha);
    }
}

ftxui::Element CameraInfoDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 5) { 
            topics_ui.push_back(ConfigHelper::render_summary(topic, configs_[topic]));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_topics_.empty() ? " No topics active" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" CameraInfo Topics ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

} // namespace terminal_rviz
