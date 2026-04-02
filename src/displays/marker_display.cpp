#include "terminal_rviz/displays/marker_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <algorithm>

namespace terminal_rviz {

MarkerDisplay::MarkerDisplay(rclcpp::Node::SharedPtr node)
    : Display("Marker", node) {}

void MarkerDisplay::onInitialize() {}

void MarkerDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        marker_subs_.erase(topic);
        marker_array_subs_.erase(topic);
        marker_store_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    TopicConfig cfg;
    cfg.color_style = "Topic"; // Default to original marker colors
    configs_[topic] = cfg;
    
    if (preferred_type_ == "visualization_msgs/msg/MarkerArray") {
        marker_array_subs_[topic] = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            topic, 10, [this, topic](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                for (const auto& marker : msg->markers) {
                    marker_store_[topic][marker.ns + std::to_string(marker.id)] = marker;
                }
            });
    } else {
        marker_subs_[topic] = node_->create_subscription<visualization_msgs::msg::Marker>(
            topic, 10, [this, topic](const visualization_msgs::msg::Marker::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                marker_store_[topic][msg->ns + std::to_string(msg->id)] = *msg;
            });
    }
}

bool MarkerDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

TopicConfig MarkerDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void MarkerDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        std::map<std::string, visualization_msgs::msg::Marker> markers;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (marker_store_.count(topic)) markers = marker_store_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        for (auto const& [key, marker] : markers) {
            if (marker.action == visualization_msgs::msg::Marker::DELETE) continue;

            tf2::Transform marker_to_world;
            try {
                auto transform_msg = tf_buffer->lookupTransform(fixed_frame, marker.header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(transform_msg.transform, marker_to_world);
            } catch (...) { continue; }

            tf2::Transform marker_pose;
            tf2::fromMsg(marker.pose, marker_pose);
            tf2::Transform total_tf = marker_to_world * marker_pose;
            tf2::Vector3 pos = total_tf.getOrigin();

            uint8_t r_c = 255, g_c = 255, b_c = 255;
            float alpha = cfg.alpha;
            if (cfg.color_style == "Topic") {
                r_c = static_cast<uint8_t>(marker.color.r * 255);
                g_c = static_cast<uint8_t>(marker.color.g * 255);
                b_c = static_cast<uint8_t>(marker.color.b * 255);
                alpha *= marker.color.a;
            } else if (cfg.color_style == "Flat") {
                static const uint8_t preset_r[] = {255, 255, 0,   0,   255, 0,   255, 255, 0,   255};
                static const uint8_t preset_g[] = {255, 0,   255, 0,   255, 255, 0,   127, 255, 127};
                static const uint8_t preset_b[] = {255, 0,   0,   255, 0,   255, 255, 0,   0,   127};
                int idx = cfg.color_index % 10;
                r_c = preset_r[idx]; g_c = preset_g[idx]; b_c = preset_b[idx];
            }

            if (marker.type == visualization_msgs::msg::Marker::CUBE) {
                renderer.draw_box(pos.x(), pos.y(), pos.z(), marker.scale.x, marker.scale.y, marker.scale.z, r_c, g_c, b_c, alpha);
            } else if (marker.type == visualization_msgs::msg::Marker::SPHERE) {
                renderer.draw_sphere(pos.x(), pos.y(), pos.z(), marker.scale.x * 0.5f, r_c, g_c, b_c, alpha);
            } else if (marker.type == visualization_msgs::msg::Marker::CYLINDER) {
                renderer.draw_sphere(pos.x(), pos.y(), pos.z(), marker.scale.x * 0.5f, r_c, g_c, b_c, alpha);
            } else if (marker.type == visualization_msgs::msg::Marker::ARROW) {
                tf2::Vector3 tip = total_tf * tf2::Vector3(marker.scale.x, 0, 0);
                renderer.draw_arrow(pos.x(), pos.y(), pos.z(), tip.x(), tip.y(), tip.z(), r_c, g_c, b_c, alpha, marker.scale.y);
            } else if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP || marker.type == visualization_msgs::msg::Marker::LINE_LIST) {
                for (size_t i = 1; i < marker.points.size(); ++i) {
                    if (marker.type == visualization_msgs::msg::Marker::LINE_LIST && i % 2 == 0) continue;
                    tf2::Vector3 p1 = total_tf * tf2::Vector3(marker.points[i-1].x, marker.points[i-1].y, marker.points[i-1].z);
                    tf2::Vector3 p2 = total_tf * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                    renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), r_c, g_c, b_c, alpha);
                }
            } else if (marker.type == visualization_msgs::msg::Marker::POINTS) {
                for (const auto& p : marker.points) {
                    tf2::Vector3 p_world = total_tf * tf2::Vector3(p.x, p.y, p.z);
                    renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), r_c, g_c, b_c, alpha);
                }
            }
        }
    }
}

ftxui::Element MarkerDisplay::render_2d(bool /*nav2_active*/, int config_scroll) {
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
        hbox({ text(" Marker Settings ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool MarkerDisplay::handle_event(ftxui::Event /*event*/, int /*scroll_offset*/) {
    return false;
}

} // namespace terminal_rviz
