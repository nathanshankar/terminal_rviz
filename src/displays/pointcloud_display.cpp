#include "terminal_rviz/displays/pointcloud_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>
#include <limits>
#include <cstring>
#include "ftxui/dom/elements.hpp"

namespace terminal_rviz {

PointCloudDisplay::PointCloudDisplay(rclcpp::Node::SharedPtr node)
    : Display("PointCloud", node) {}

void PointCloudDisplay::onInitialize() {
}

void PointCloudDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        subs_.erase(topic);
        latest_clouds_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg; 
    cfg.size = 0.0f; 
    cfg.alpha = 1.0f;
    configs_[topic] = cfg;
    
    try {
        subs_[topic] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->callback(msg, topic);
            });
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool PointCloudDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void PointCloudDisplay::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_clouds_[topic] = msg;
}

TopicConfig PointCloudDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void PointCloudDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void PointCloudDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_clouds_.count(topic)) msg = latest_clouds_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        if (!msg) continue;

        tf2::Transform pc_to_world;
        try {
            auto transform_msg = tf_buffer->lookupTransform(fixed_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(transform_msg.transform, pc_to_world);
        } catch (...) { continue; }

        bool has_rgb = false;
        try {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> test_rgb(*msg, "rgb");
            has_rgb = true;
        } catch (...) {
            try {
                sensor_msgs::PointCloud2ConstIterator<uint8_t> test_rgba(*msg, "rgba");
                has_rgb = true;
            } catch (...) {}
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
        
        // Use local variable for iter_rgb to avoid scope issues
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_rgb_ptr;
        if (has_rgb) {
            try {
                iter_rgb_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "rgb");
            } catch (...) {
                try {
                    iter_rgb_ptr = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*msg, "rgba");
                } catch (...) { has_rgb = false; }
            }
        }

        size_t total_points = msg->width * msg->height;
        if (total_points == 0) continue;

        size_t max_render = 20000;
        size_t skip = std::max((size_t)1, total_points / max_render);

        float min_val = 0.0f, max_val = 1.0f;
        float range_inv = 1.0f;

        if (cfg.color_style == "Axis") {
            min_val = std::numeric_limits<float>::max();
            max_val = std::numeric_limits<float>::lowest();
            bool found_any = false;
            auto r_iter_x = iter_x; auto r_iter_y = iter_y; auto r_iter_z = iter_z;
            for (size_t i = 0; i < total_points; i += skip, r_iter_x += skip, r_iter_y += skip, r_iter_z += skip) {
                float val = (cfg.axis == "Y") ? *r_iter_y : ((cfg.axis == "Z") ? *r_iter_z : *r_iter_x);
                if (std::isfinite(val)) {
                    if (val < min_val) min_val = val;
                    if (val > max_val) max_val = val;
                    found_any = true;
                }
            }
            if (!found_any) { min_val = 0.0f; max_val = 1.0f; }
            if (std::abs(max_val - min_val) < 0.001f) max_val = min_val + 1.0f;
            range_inv = 1.0f / (max_val - min_val);
        }

        for (size_t i = 0; i < total_points; i += skip, iter_x += skip, iter_y += skip, iter_z += skip) {
            tf2::Vector3 p_world = pc_to_world * tf2::Vector3(*iter_x, *iter_y, *iter_z);
            int sx, sy; float sz;
            if (renderer.project(p_world.x(), p_world.y(), p_world.z(), sx, sy, sz)) {
                uint8_t r_c = 255, g_c = 255, b_c = 255;
                
                if (cfg.color_style == "RGB" && has_rgb && iter_rgb_ptr) {
                    auto it_val = (*iter_rgb_ptr) + i;
                    r_c = it_val[2]; g_c = it_val[1]; b_c = it_val[0];
                } else if (cfg.color_style == "Flat") {
                    // Pre-defined high-fidelity preset colors
                    static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
                    static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
                    static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
                    int idx = cfg.color_index % 10;
                    r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
                } else {
                    float val = (cfg.axis == "Y") ? *iter_y : ((cfg.axis == "Z") ? *iter_z : *iter_x);
                    float v = std::clamp((val - min_val) * range_inv, 0.0f, 1.0f);
                    if (v < 0.25f) { r_c = 255; g_c = static_cast<uint8_t>(v * 1020); b_c = 0; }
                    else if (v < 0.5f) { r_c = static_cast<uint8_t>((0.5f - v) * 1020); g_c = 255; b_c = 0; }
                    else if (v < 0.75f) { r_c = 0; g_c = 255; b_c = static_cast<uint8_t>((v - 0.5f) * 1020); }
                    else { r_c = 0; g_c = static_cast<uint8_t>((1.0f - v) * 1020); b_c = 255; }
                }
                
                r_c = static_cast<uint8_t>(r_c * cfg.alpha);
                g_c = static_cast<uint8_t>(g_c * cfg.alpha);
                b_c = static_cast<uint8_t>(b_c * cfg.alpha);
                
                int r = (int)(cfg.size * 10.0f);
                if (r <= 0) renderer.plot(sx, sy, sz, ftxui::Color::RGB(r_c, g_c, b_c));
                else for (int dy = -r; dy <= r; ++dy) for (int dx = -r; dx <= r; ++dx) renderer.plot(sx + dx, sy + dy, sz, ftxui::Color::RGB(r_c, g_c, b_c));
            }
        }
    }
}

ftxui::Element PointCloudDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" PointCloud Topics ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool PointCloudDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;
    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12); 
    if (ry < 0 || ry >= 10) return false;
    int item_idx = ry + scroll_offset;
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_topics_.size()) return true; 
    return false;
}

} // namespace terminal_rviz
