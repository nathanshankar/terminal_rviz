#include "terminal_rviz/displays/effort_display.hpp"
#include "terminal_rviz/config_helper.hpp"
#include <iomanip>
#include <sstream>

namespace terminal_rviz {

EffortDisplay::EffortDisplay(rclcpp::Node::SharedPtr node)
    : Display("Effort", node) {}

void EffortDisplay::setTopic(const std::string& topic) {
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
    configs_[topic] = TopicConfig();
    subs_[topic] = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic, 10, [this, topic](const sensor_msgs::msg::JointState::SharedPtr msg) {
            callback(msg, topic);
        });
}

bool EffortDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void EffortDisplay::callback(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_msgs_[topic] = msg;
}

TopicConfig EffortDisplay::getTopicConfig(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (configs_.count(topic)) return configs_[topic];
    return TopicConfig();
}

void EffortDisplay::setTopicConfig(const std::string& topic, const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mtx_);
    configs_[topic] = config;
}

void EffortDisplay::render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) {
    // Efforts are mainly shown in the 2D panel as a summary for now
}

ftxui::Element EffortDisplay::render_2d(bool, int config_scroll) {
    using namespace ftxui;
    std::lock_guard<std::mutex> lock(mtx_);
    Elements topics_ui;
    int count = 0;
    for (const auto& topic : enabled_topics_) {
        if (count >= config_scroll && count < config_scroll + 5) {
            Elements joint_efforts;
            if (latest_msgs_.count(topic)) {
                auto msg = latest_msgs_[topic];
                for (size_t i = 0; i < std::min((size_t)10, msg->name.size()); ++i) {
                    double effort = (i < msg->effort.size()) ? msg->effort[i] : 0.0;
                    joint_efforts.push_back(hbox({
                        text("  " + msg->name[i] + ": ") | size(WIDTH, EQUAL, 20),
                        text(std::to_string(effort)) | color(Color::Cyan)
                    }));
                }
            } else {
                joint_efforts.push_back(text("  No data yet") | dim);
            }

            topics_ui.push_back(vbox({
                hbox({ text(" Topic: ") | bold, text(topic) | color(Color::Green) }),
                vbox(std::move(joint_efforts)),
                separator()
            }));
        }
        count++;
    }
    if (topics_ui.empty()) return text(enabled_topics_.empty() ? " No topics active" : " (End of list)") | dim | center;
    return vbox({
        hbox({ text(" Joint Effort Summary ") | bold | color(Color::Yellow), filler() }),
        separator(),
        vbox(std::move(topics_ui)) | size(HEIGHT, EQUAL, 10) | vscroll_indicator | frame,
    }) | border;
}

bool EffortDisplay::handle_event(ftxui::Event event, int scroll_offset) {
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
