#include "terminal_rviz/displays/marker_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    
    marker_array_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic_ + "_array", 10, std::bind(&MarkerDisplay::markerArrayCallback, this, std::placeholders::_1));
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

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<visualization_msgs::msg::Marker> markers_to_render;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& pair : markers_) {
            markers_to_render.push_back(pair.second);
        }
    }

    for (const auto& marker : markers_to_render) {
        tf2::Transform marker_to_world;
        try {
            auto transform_msg = tf_buffer->lookupTransform(fixed_frame, marker.header.frame_id, tf2::TimePointZero);
            
            // Manual conversion if fromMsg linker fails
            const auto& t = transform_msg.transform.translation;
            const auto& r = transform_msg.transform.rotation;
            marker_to_world.setOrigin(tf2::Vector3(t.x, t.y, t.z));
            marker_to_world.setRotation(tf2::Quaternion(r.x, r.y, r.z, r.w));
        } catch (...) { continue; }

        ftxui::Color color = ftxui::Color::RGB(
            static_cast<uint8_t>(marker.color.r * 255),
            static_cast<uint8_t>(marker.color.g * 255),
            static_cast<uint8_t>(marker.color.b * 255));

        tf2::Transform visual_to_marker;
        const auto& p = marker.pose.position;
        const auto& q = marker.pose.orientation;
        visual_to_marker.setOrigin(tf2::Vector3(p.x, p.y, p.z));
        visual_to_marker.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
        
        tf2::Transform visual_to_world = marker_to_world * visual_to_marker;

        auto draw_point_world = [&](const tf2::Vector3& p_local) {
            tf2::Vector3 p_w = visual_to_world * p_local;
            renderer.draw_point(p_w.x(), p_w.y(), p_w.z(), color, canvas);
        };

        switch (marker.type) {
            case visualization_msgs::msg::Marker::CUBE:
            case visualization_msgs::msg::Marker::SPHERE:
            case visualization_msgs::msg::Marker::CYLINDER:
                draw_point_world(tf2::Vector3(0,0,0));
                break;
            
            case visualization_msgs::msg::Marker::LINE_STRIP:
                if (marker.points.size() >= 2) {
                    for (size_t i = 0; i < marker.points.size() - 1; ++i) {
                        tf2::Vector3 p1_local(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                        tf2::Vector3 p2_local(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                        tf2::Vector3 p1 = visual_to_world * p1_local;
                        tf2::Vector3 p2 = visual_to_world * p2_local;
                        renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), color, canvas);
                    }
                }
                break;

            case visualization_msgs::msg::Marker::POINTS:
                for (const auto& pt : marker.points) {
                    draw_point_world(tf2::Vector3(pt.x, pt.y, pt.z));
                }
                break;
        }
    }
}

} // namespace terminal_rviz
