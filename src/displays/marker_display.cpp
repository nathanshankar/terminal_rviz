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
    try {
        marker_sub_.reset();
        marker_array_sub_.reset();
        topic_ = topic;
        marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
            topic_, 10, std::bind(&MarkerDisplay::markerCallback, this, std::placeholders::_1));
        marker_array_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            topic_ + "_array", 10, std::bind(&MarkerDisplay::markerArrayCallback, this, std::placeholders::_1));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Marker: Failed to subscribe to %s: %s", topic.c_str(), e.what());
    }
}

void MarkerDisplay::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::string key = msg->ns + "_" + std::to_string(msg->id);
    if (msg->action == visualization_msgs::msg::Marker::DELETE) markers_.erase(key);
    else if (msg->action == visualization_msgs::msg::Marker::DELETEALL) markers_.clear();
    else markers_[key] = *msg;
}

void MarkerDisplay::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    for (const auto& m : msg->markers) markerCallback(std::make_shared<visualization_msgs::msg::Marker>(m));
}

void MarkerDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    std::vector<visualization_msgs::msg::Marker> to_render;
    { std::lock_guard<std::mutex> lock(mtx_); for (auto const& [k, v] : markers_) to_render.push_back(v); }

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

        auto draw_pt_w = [&](const tf2::Vector3& p_l) {
            tf2::Vector3 p = v_to_w * p_l;
            renderer.draw_point(p.x(), p.y(), p.z(), col);
        };

        switch (marker.type) {
            case visualization_msgs::msg::Marker::CUBE:
            case visualization_msgs::msg::Marker::SPHERE:
            case visualization_msgs::msg::Marker::CYLINDER: draw_pt_w(tf2::Vector3(0,0,0)); break;
            case visualization_msgs::msg::Marker::LINE_STRIP:
                if (marker.points.size() >= 2) for (size_t i=0; i<marker.points.size()-1; ++i) {
                    tf2::Vector3 p1 = v_to_w * tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
                    tf2::Vector3 p2 = v_to_w * tf2::Vector3(marker.points[i+1].x, marker.points[i+1].y, marker.points[i+1].z);
                    renderer.draw_line(p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), col);
                } break;
            case visualization_msgs::msg::Marker::POINTS: for (auto const& p : marker.points) draw_pt_w(tf2::Vector3(p.x, p.y, p.z)); break;
        }
    }
}

} // namespace terminal_rviz
