#include "terminal_rviz/displays/teleop_display.hpp"
#include <algorithm>
#include <cmath>
#include "ftxui/dom/elements.hpp"

namespace terminal_rviz {

TeleopDisplay::TeleopDisplay(rclcpp::Node::SharedPtr node)
    : Display("Teleop", node) {}

void TeleopDisplay::onInitialize() {
    // Default topic if none provided
    if (topic_.empty()) {
        topic_ = "/cmd_vel";
    }
    setTopic(topic_);
}

void TeleopDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    topic_ = topic;
    try {
        pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
    } catch (...) {
        pub_ = nullptr;
    }
}

void TeleopDisplay::publish_twist(double linear_x, double angular_z) {
    if (!pub_) return;
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    pub_->publish(msg);
}

void TeleopDisplay::render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) {
    // Nothing to render in 3D
}

ftxui::Element TeleopDisplay::render_2d(bool /*nav2_active*/, int /*config_scroll*/) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);

    auto speed_info = hbox({
        text(" Speed: ") | bold,
        text(std::to_string(speed_).substr(0, 4)) | color(Color::Green),
        text("  Turn: ") | bold,
        text(std::to_string(turn_).substr(0, 4)) | color(Color::Cyan),
    });

    auto key_map = vbox({
        hbox({ text("  u  i  o  ") | bold | color(Color::Yellow), text("   [q/z]: Speed +/-") | dim }),
        hbox({ text("  j  k  l  ") | bold | color(Color::Yellow), text("   [w/x]: Turn  +/-") | dim }),
        hbox({ text("  m  ,  .  ") | bold | color(Color::Yellow), text("   [k/Spc]: STOP") | dim }),
    });

    return vbox({
        hbox({ text(" Teleop Keyboard ") | bold | color(Color::Magenta), filler() }),
        separator(),
        text(" Topic: " + topic_) | color(Color::Cyan),
        separator(),
        speed_info,
        separator(),
        key_map,
        separator(),
        hbox({ text(" Last Command: ") | bold, text(last_key_) | color(Color::GreenLight) }),
    }) | border | size(HEIGHT, EQUAL, 14);
}

bool TeleopDisplay::handle_event(ftxui::Event event, int /*scroll_offset*/) {
    if (!enabled_ || !added_) return false;

    std::lock_guard<std::mutex> lock(mtx_);
    
    // Standard teleop keys
    static std::map<std::string, std::pair<double, double>> move_bindings = {
        {"i", {1, 0}},   {"u", {1, 1}},   {"o", {1, -1}},
        {"j", {0, 1}},   {"k", {0, 0}},   {"l", {0, -1}},
        {"m", {-1, -1}}, {",", {-1, 0}},  {".", {-1, 1}},
    };

    if (event.is_character()) {
        std::string key = event.character();
        
        if (move_bindings.count(key)) {
            x_ = move_bindings[key].first;
            th_ = move_bindings[key].second;
            last_key_ = "Move: " + key;
            publish_twist(x_ * speed_, th_ * turn_);
            return true;
        } else if (key == "q") {
            speed_ *= 1.1;
            last_key_ = "Speed Up";
            return true;
        } else if (key == "z") {
            speed_ *= 0.9;
            last_key_ = "Speed Down";
            return true;
        } else if (key == "w") {
            turn_ *= 1.1;
            last_key_ = "Turn Up";
            return true;
        } else if (key == "x") {
            turn_ *= 0.9;
            last_key_ = "Turn Down";
            return true;
        } else if (key == " ") {
            x_ = 0; th_ = 0;
            last_key_ = "STOP";
            publish_twist(0, 0);
            return true;
        }
    }

    return false;
}

} // namespace terminal_rviz
