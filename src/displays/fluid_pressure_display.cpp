#include "terminal_rviz/displays/fluid_pressure_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace terminal_rviz {

FluidPressureDisplay::FluidPressureDisplay(rclcpp::Node::SharedPtr node)
    : Display("FluidPressure", node) {}

void FluidPressureDisplay::setTopic(const std::string& topic) {
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
    cfg.color_index = 5; // Cyan default
    cfg.size = 0.1f;
    cfg.style = "Points";
    configs_[topic] = cfg;
    
    subs_[topic] = node_->create_subscription<sensor_msgs::msg::FluidPressure>(
        topic, 10, [this, topic](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_msgs_[topic] = msg;
        });
}

bool FluidPressureDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig FluidPressureDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void FluidPressureDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void FluidPressureDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        sensor_msgs::msg::FluidPressure::SharedPtr msg;
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
        
        tf2::Vector3 p = frame_to_world * tf2::Vector3(0, 0, 0);

        render_styled_point(renderer, p.x(), p.y(), p.z(), cfg, r_c, g_c, b_c);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << msg->fluid_pressure << " Pa";
        renderer.draw_text(p.x(), p.y(), p.z() + 0.1f, ss.str(), ftxui::Color::RGB(r_c, g_c, b_c));
    }
}

ftxui::Element FluidPressureDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" Pressure Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool FluidPressureDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
