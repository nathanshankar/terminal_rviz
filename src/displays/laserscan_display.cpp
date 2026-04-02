#include "terminal_rviz/displays/laserscan_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

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
        latest_msgs_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg;
    cfg.color_index = 1; // Red default
    cfg.size = 0.05f;
    cfg.style = "Points";
    configs_[topic] = cfg;
    
    subs_[topic] = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, 10, [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_msgs_[topic] = msg;
        });
}

bool LaserScanDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
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

void LaserScanDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
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
            if (latest_msgs_.count(topic)) msg = latest_msgs_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform frame_to_world;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(t_msg.transform, frame_to_world);
        } catch (...) { continue; }

        auto projector = renderer.get_projector(frame_to_world);
        const tf2::Matrix3x3& rot = frame_to_world.getBasis();
        const tf2::Vector3& trans = frame_to_world.getOrigin();

        uint8_t r_b = 255, g_b = 255, b_b = 255;
        if (cfg.color_style == "Flat") {
            static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
            static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
            static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
            int idx = cfg.color_index % 10;
            r_b = preset_r[idx]; g_b = preset_g[idx]; b_b = preset_b[idx];
        }
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;
            
            float angle = msg->angle_min + i * msg->angle_increment;
            float lx = r * std::cos(angle);
            float ly = r * std::sin(angle);

            int sx, sy; float sz;
            if (!projector.project(lx, ly, 0, sx, sy, sz)) continue;
            
            uint8_t r_c = r_b, g_c = g_b, b_c = b_b;
            if (cfg.color_style == "Axis") {
                float wx = rot[0][0] * lx + rot[0][1] * ly + trans.x();
                float wy = rot[1][0] * lx + rot[1][1] * ly + trans.y();
                float wz = rot[2][0] * lx + rot[2][1] * ly + trans.z();
                float val = (cfg.axis == "X") ? wx : ((cfg.axis == "Y") ? wy : wz);
                float v = std::clamp((val + 2.0f) / 4.0f, 0.0f, 1.0f);
                if (v < 0.25f) { r_c = 255; g_c = static_cast<uint8_t>(v * 1020); b_c = 0; }
                else if (v < 0.5f) { r_c = static_cast<uint8_t>((0.5f - v) * 1020); g_c = 255; b_c = 0; }
                else if (v < 0.75f) { r_c = 0; g_c = 255; b_c = static_cast<uint8_t>((v - 0.5f) * 1020); }
                else { r_c = 0; g_c = static_cast<uint8_t>((1.0f - v) * 1020); b_c = 255; }
            }

            if (cfg.style == "Points") {
                renderer.plot(sx, sy, sz, r_c, g_c, b_c, cfg.alpha);
            } else {
                float wx = rot[0][0] * lx + rot[0][1] * ly + trans.x();
                float wy = rot[1][0] * lx + rot[1][1] * ly + trans.y();
                float wz = rot[2][0] * lx + rot[2][1] * ly + trans.z();
                render_styled_point(renderer, wx, wy, wz, cfg, r_c, g_c, b_c);
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
            topics_ui.push_back(ConfigHelper::render_summary(topic, configs_[topic]));
        }
        count++;
    }
    return vbox({
        hbox({ text(" LaserScan Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool LaserScanDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
