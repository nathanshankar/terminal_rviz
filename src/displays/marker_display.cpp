#include "terminal_rviz/displays/marker_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <cmath>
#include <algorithm>

namespace terminal_rviz {

MarkerDisplay::MarkerDisplay(rclcpp::Node::SharedPtr node)
    : Display("Markers", node) {}

void MarkerDisplay::onInitialize() {
}

void MarkerDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        marker_subs_.erase(topic);
        marker_array_subs_.erase(topic);
        markers_.erase(topic);
        configs_.erase(topic);
        return;
    }
    
    enabled_topics_.push_back(topic);
    configs_[topic] = TopicConfig();
    
    try {
        if (preferred_type_ == "visualization_msgs/msg/MarkerArray") {
            marker_array_subs_[topic] = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
                topic, 10, [this, topic](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    this->markerArrayCallback(msg, topic);
                });
        } else {
            marker_subs_[topic] = node_->create_subscription<visualization_msgs::msg::Marker>(
                topic, 10, [this, topic](const visualization_msgs::msg::Marker::SharedPtr msg) {
                    this->markerCallback(msg, topic);
                });
        }
    } catch (...) {
        enabled_topics_.pop_back();
    }
}

bool MarkerDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void MarkerDisplay::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::string key = msg->ns + "_" + std::to_string(msg->id);
    if (msg->action == visualization_msgs::msg::Marker::DELETE) {
        markers_[topic].erase(key);
    } else if (msg->action == visualization_msgs::msg::Marker::DELETEALL) {
        markers_[topic].clear();
    } else {
        markers_[topic][key] = *msg;
    }
}

void MarkerDisplay::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, const std::string& topic) {
    for (const auto& m : msg->markers) {
        markerCallback(std::make_shared<visualization_msgs::msg::Marker>(m), topic);
    }
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

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<std::string> topics;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics = enabled_topics_;
    }

    for (const auto& topic : topics) {
        std::map<std::string, visualization_msgs::msg::Marker> to_render;
        TopicConfig cfg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (markers_.count(topic)) to_render = markers_[topic];
            if (configs_.count(topic)) cfg = configs_[topic];
        }

        for (auto const& [k, marker] : to_render) {
            tf2::Transform m_to_w;
            try {
                auto t_msg = tf_buffer->lookupTransform(fixed_frame, marker.header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(t_msg.transform, m_to_w);
            } catch (...) { continue; }

            ftxui::Color col;
            if (cfg.color_style == "Flat") col = ftxui::Color::RGB(cfg.r, cfg.g, cfg.b);
            else col = ftxui::Color::RGB(int(marker.color.r*255), int(marker.color.g*255), int(marker.color.b*255));

            tf2::Transform v_to_m;
            tf2::fromMsg(marker.pose, v_to_m);
            tf2::Transform v_to_w = m_to_w * v_to_m;

            auto draw_l = [&](float x1, float y1, float z1, float x2, float y2, float z2) {
                tf2::Vector3 p1 = v_to_w * tf2::Vector3(x1, y1, z1);
                tf2::Vector3 p2 = v_to_w * tf2::Vector3(x2, y2, z2);
                renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
            };

            float sx = marker.scale.x * 0.5f;
            float sy = marker.scale.y * 0.5f;
            float sz = marker.scale.z * 0.5f;

            switch (marker.type) {
                case visualization_msgs::msg::Marker::CUBE:
                    draw_l(-sx, -sy, -sz,  sx, -sy, -sz); draw_l( sx, -sy, -sz,  sx,  sy, -sz);
                    draw_l( sx,  sy, -sz, -sx,  sy, -sz); draw_l(-sx,  sy, -sz, -sx, -sy, -sz);
                    draw_l(-sx, -sy,  sz,  sx, -sy,  sz); draw_l( sx, -sy,  sz,  sx,  sy,  sz);
                    draw_l( sx,  sy,  sz, -sx,  sy,  sz); draw_l(-sx,  sy,  sz, -sx, -sy,  sz);
                    draw_l(-sx, -sy, -sz, -sx, -sy,  sz); draw_l( sx, -sy, -sz,  sx, -sy,  sz);
                    draw_l( sx,  sy, -sz,  sx,  sy,  sz); draw_l(-sx,  sy, -sz, -sx,  sy,  sz);
                    break;
                case visualization_msgs::msg::Marker::SPHERE: {
                    int steps = 8;
                    for (int i=0; i<steps; ++i) {
                        float a1 = 2.0f * M_PI * i / steps;
                        float a2 = 2.0f * M_PI * (i+1) / steps;
                        draw_l(sx*cos(a1), sy*sin(a1), 0, sx*cos(a2), sy*sin(a2), 0);
                        draw_l(sx*cos(a1), 0, sz*sin(a1), sx*cos(a2), 0, sz*sin(a2));
                        draw_l(0, sy*cos(a1), sz*sin(a1), 0, sy*cos(a2), sz*sin(a2));
                    }
                    break;
                }
                case visualization_msgs::msg::Marker::LINE_STRIP:
                    for (size_t i=0; i+1 < marker.points.size(); ++i) {
                        tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                        tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                        renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                    }
                    break;
                case visualization_msgs::msg::Marker::LINE_LIST:
                    for (size_t i=0; i+1 < marker.points.size(); i+=2) {
                        tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                        tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                        renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                    }
                    break;
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
        if (count >= config_scroll && count < config_scroll + 5) { 
            auto& cfg = configs_[topic];
            topics_ui.push_back(vbox({
                hbox({ text(" Topic: ") | bold, text(topic) | color(Color::Green) }),
                hbox({
                    text(" Color Style: "),
                    text(" [" + cfg.color_style + "] ") | color(Color::Cyan),
                }),
                separator(),
            }));
        }
        count++;
    }
    
    if (topics_ui.empty()) {
        if (enabled_topics_.empty()) return text(" No Marker topics active") | dim | center;
        return text(" (End of list)") | dim | center;
    }

    return vbox({
        hbox({ text(" Marker Settings ") | bold | color(Color::Yellow), filler(), text(" [Scroll/Toggle] ") | dim }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10),
    }) | border;
}

bool MarkerDisplay::handle_event(ftxui::Event event, int scroll_offset) {
    if (!event.is_mouse()) return false;
    auto mouse = event.mouse();
    if (mouse.button != ftxui::Mouse::Left || mouse.motion != ftxui::Mouse::Pressed) return false;

    auto terminal = ftxui::Terminal::Size();
    int ry = mouse.y - (terminal.dimy - 12); 
    if (ry < 0 || ry >= 10) return false;

    int item_idx = (ry / 2) + scroll_offset;
    
    std::lock_guard<std::mutex> lock(mtx_);
    if (item_idx >= 0 && item_idx < (int)enabled_topics_.size()) {
        std::string topic = enabled_topics_[item_idx];
        auto& cfg = configs_[topic];
        if (cfg.color_style == "Flat") cfg.color_style = "Original";
        else cfg.color_style = "Flat";
        return true;
    }
    return false;
}

} // namespace terminal_rviz
