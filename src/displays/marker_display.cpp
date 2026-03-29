#include "terminal_rviz/displays/marker_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace terminal_rviz {

MarkerDisplay::MarkerDisplay(rclcpp::Node::SharedPtr node)
    : Display("Markers", node) {}

void MarkerDisplay::onInitialize() {
    std::string default_topic = (preferred_type_ == "visualization_msgs/msg/MarkerArray") ? "marker_array" : "marker";
    topic_ = node_->declare_parameter(name_ + ".topic", default_topic);
    setTopic(topic_);
}

void MarkerDisplay::setTopic(const std::string& topic) {
    try {
        marker_sub_.reset();
        marker_array_sub_.reset();
        topic_ = topic;
        
        if (preferred_type_ == "visualization_msgs/msg/MarkerArray") {
            marker_array_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
                topic_, 10, std::bind(&MarkerDisplay::markerArrayCallback, this, std::placeholders::_1));
        } else {
            marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
                topic_, 10, std::bind(&MarkerDisplay::markerCallback, this, std::placeholders::_1));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Marker: Failed to subscribe to %s: %s", topic.c_str(), e.what());
    }
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
    for (const auto& m : msg->markers) {
        markerCallback(std::make_shared<visualization_msgs::msg::Marker>(m));
    }
}

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<visualization_msgs::msg::Marker> to_render;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto const& [k, v] : markers_) to_render.push_back(v);
    }

    for (const auto& marker : to_render) {
        tf2::Transform m_to_w;
        try {
            auto t_msg = tf_buffer->lookupTransform(fixed_frame, marker.header.frame_id, tf2::TimePointZero);
            m_to_w.setOrigin(tf2::Vector3(t_msg.transform.translation.x, t_msg.transform.translation.y, t_msg.transform.translation.z));
            m_to_w.setRotation(tf2::Quaternion(t_msg.transform.rotation.x, t_msg.transform.rotation.y, t_msg.transform.rotation.z, t_msg.transform.rotation.w));
        } catch (...) { continue; }

        auto col = ftxui::Color::RGB(int(marker.color.r*255), int(marker.color.g*255), int(marker.color.b*255));
        tf2::Transform v_to_m;
        v_to_m.setOrigin(tf2::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
        v_to_m.setRotation(tf2::Quaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w));
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
                    draw_l(marker.scale.x*0.5f*cos(a1), marker.scale.y*0.5f*sin(a1), 0, marker.scale.x*0.5f*cos(a2), marker.scale.y*0.5f*sin(a2), 0);
                    draw_l(marker.scale.x*0.5f*cos(a1), 0, marker.scale.z*0.5f*sin(a1), marker.scale.x*0.5f*cos(a2), 0, marker.scale.z*0.5f*sin(a2));
                    draw_l(0, marker.scale.y*0.5f*cos(a1), marker.scale.z*0.5f*sin(a1), 0, marker.scale.y*0.5f*cos(a2), marker.scale.z*0.5f*sin(a2));
                }
                break;
            }

            case visualization_msgs::msg::Marker::CYLINDER: {
                int steps = 8;
                for (int i=0; i<steps; ++i) {
                    float a1 = 2.0f * M_PI * i / steps;
                    float a2 = 2.0f * M_PI * (i+1) / steps;
                    draw_l(sx*cos(a1), sy*sin(a1), -sz, sx*cos(a2), sy*sin(a2), -sz);
                    draw_l(sx*cos(a1), sy*sin(a1),  sz, sx*cos(a2), sy*sin(a2),  sz);
                    if (i % 2 == 0) draw_l(sx*cos(a1), sy*sin(a1), -sz, sx*cos(a1), sy*sin(a1), sz);
                }
                break;
            }

            case visualization_msgs::msg::Marker::ARROW:
                if (marker.points.size() >= 2) {
                    tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[0].x, marker.points[0].y, marker.points[0].z);
                    tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[1].x, marker.points[1].y, marker.points[1].z);
                    renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                    // Simple tip
                    tf2::Vector3 dir = (p2 - p1).normalized();
                    tf2::Vector3 side = dir.cross(tf2::Vector3(0,0,1)).normalized() * 0.2f * marker.scale.y;
                    renderer.draw_line(p2.x(), p2.y(), p2.z(), p2.x() - dir.x()*0.3f*marker.scale.x + side.x(), p2.y() - dir.y()*0.3f*marker.scale.x + side.y(), p2.z() - dir.z()*0.3f*marker.scale.x + side.z(), col);
                    renderer.draw_line(p2.x(), p2.y(), p2.z(), p2.x() - dir.x()*0.3f*marker.scale.x - side.x(), p2.y() - dir.y()*0.3f*marker.scale.x - side.y(), p2.z() - dir.z()*0.3f*marker.scale.x - side.z(), col);
                } else {
                    draw_l(0,0,0, marker.scale.x, 0, 0);
                    draw_l(marker.scale.x, 0, 0, marker.scale.x*0.8f,  0.1f*marker.scale.y, 0);
                    draw_l(marker.scale.x, 0, 0, marker.scale.x*0.8f, -0.1f*marker.scale.y, 0);
                }
                break;

            case visualization_msgs::msg::Marker::LINE_STRIP:
                if (marker.points.size() >= 2) {
                    for (size_t i=0; i<marker.points.size()-1; ++i) {
                        tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                        tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                        renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                    }
                }
                break;

            case visualization_msgs::msg::Marker::LINE_LIST:
                for (size_t i=0; i+1 < marker.points.size(); i+=2) {
                    tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                    tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                    renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                }
                break;

            case visualization_msgs::msg::Marker::POINTS:
                for (auto const& p : marker.points) {
                    tf2::Vector3 pw = v_to_w * tf2::Vector3(p.x, p.y, p.z);
                    renderer.draw_point(pw.x(), pw.y(), pw.z(), col);
                }
                break;
        }
    }
}

} // namespace terminal_rviz
