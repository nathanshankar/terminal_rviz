#include "terminal_rviz/displays/marker_display.hpp"

namespace terminal_rviz {

MarkerDisplay::MarkerDisplay(rclcpp::Node::SharedPtr node)
    : Display("Markers", node) {}

void MarkerDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "marker");
    setTopic(topic_);
}

void MarkerDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
        topic_, 10, std::bind(&MarkerDisplay::markerCallback, this, std::placeholders::_1));
    
    // For MarkerArray, we can optionally have another topic, 
    // but usually they go together or are remapped separately.
}

void MarkerDisplay::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::string key = msg->ns + "_" + std::to_string(msg->id);
    if (msg->action == visualization_msgs::msg::Marker::DELETE) {
        markers_.erase(key);
    } else if (msg->action == visualization_msgs::msg::Marker::DELETEALL) {
        markers_.clear();
    } else {
        markers_[key] = *msg;
    }
}

void MarkerDisplay::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    for (const auto& marker : msg->markers) {
        auto m_ptr = std::make_shared<visualization_msgs::msg::Marker>(marker);
        markerCallback(m_ptr);
    }
}

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    std::vector<visualization_msgs::msg::Marker> markers_to_render;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& pair : markers_) {
            markers_to_render.push_back(pair.second);
        }
    }

    for (const auto& marker : markers_to_render) {
        ftxui::Color color = ftxui::Color::RGB(
            static_cast<uint8_t>(marker.color.r * 255),
            static_cast<uint8_t>(marker.color.g * 255),
            static_cast<uint8_t>(marker.color.b * 255));

        switch (marker.type) {
            case visualization_msgs::msg::Marker::CUBE:
            case visualization_msgs::msg::Marker::SPHERE:
            case visualization_msgs::msg::Marker::CYLINDER:
                renderer.draw_point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, color, canvas);
                break;
            
            case visualization_msgs::msg::Marker::LINE_STRIP:
                if (marker.points.size() >= 2) {
                    for (size_t i = 0; i < marker.points.size() - 1; ++i) {
                        renderer.draw_line(
                            marker.points[i].x, marker.points[i].y, marker.points[i].z,
                            marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z,
                            color, canvas);
                    }
                }
                break;

            case visualization_msgs::msg::Marker::LINE_LIST:
                if (marker.points.size() >= 2) {
                    for (size_t i = 0; i < marker.points.size() - 1; i += 2) {
                        renderer.draw_line(
                            marker.points[i].x, marker.points[i].y, marker.points[i].z,
                            marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z,
                            color, canvas);
                    }
                }
                break;
            
            case visualization_msgs::msg::Marker::POINTS:
                for (const auto& p : marker.points) {
                    renderer.draw_point(p.x, p.y, p.z, color, canvas);
                }
                break;
        }
    }
}

} // namespace terminal_rviz
