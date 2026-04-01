#include "terminal_rviz/displays/image_display.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/dom/canvas.hpp"
#include <algorithm>
#include <cstring>

namespace terminal_rviz {

ImageDisplay::ImageDisplay(rclcpp::Node::SharedPtr node)
    : Display("Image", node) {}

void ImageDisplay::onInitialize() {}

void ImageDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic);
    if (it != enabled_topics_.end()) {
        enabled_topics_.erase(it);
        
        // Check if anyone else is still using this topic
        bool still_used = std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
        if (!still_used) {
            subs_.erase(topic);
            latest_images_.erase(topic);
        }
        return;
    }
    if (enabled_topics_.size() >= 2) return;
    enabled_topics_.push_back(topic);
    if (subs_.find(topic) == subs_.end()) {
        try {
            subs_[topic] = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, 10, [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->callback(msg, topic);
                });
        } catch (...) {
            enabled_topics_.pop_back();
        }
    }
}

void ImageDisplay::replaceTopic(int slot, const std::string& new_topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (slot < 0 || slot >= (int)enabled_topics_.size()) return;
    
    std::string old_topic = enabled_topics_[slot];
    if (old_topic == new_topic) return;

    // Remove old slot reference
    enabled_topics_[slot] = new_topic;

    // Clean up old subscription only if no other slots use it
    bool old_still_used = std::find(enabled_topics_.begin(), enabled_topics_.end(), old_topic) != enabled_topics_.end();
    if (!old_still_used) {
        subs_.erase(old_topic);
        latest_images_.erase(old_topic);
    }

    // Create new subscription if needed
    if (subs_.find(new_topic) == subs_.end()) {
        try {
            subs_[new_topic] = node_->create_subscription<sensor_msgs::msg::Image>(
                new_topic, 10, [this, new_topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->callback(msg, new_topic);
                });
        } catch (...) {
            // Fallback?
        }
    }
}

bool ImageDisplay::isTopicEnabled(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::find(enabled_topics_.begin(), enabled_topics_.end(), topic) != enabled_topics_.end();
}

void ImageDisplay::callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_images_[topic] = msg;
}

void ImageDisplay::render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) {}

ftxui::Element ImageDisplay::render_2d(bool nav2_active, int /*config_scroll*/) {
    if (!enabled_ || enabled_topics_.empty()) return ftxui::filler();

    ftxui::Elements panels;
    std::vector<std::string> topics_copy;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        topics_copy = enabled_topics_;
    }

    auto terminal = ftxui::Terminal::Size();
    int num_images = topics_copy.size();
    int available_char_h = std::max(10, terminal.dimy - 4); 
    if (nav2_active) available_char_h -= 10;
    int char_h_per_img = (available_char_h / num_images); 
    int image_rows = char_h_per_img - 3;
    
    int target_h = std::max(4, image_rows * 4);
    int target_w = (target_h * 4) / 3;
    if (target_w > 120) { target_w = 120; target_h = (target_w * 3) / 4; }

    for (const auto& topic : topics_copy) {
        sensor_msgs::msg::Image::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (latest_images_.count(topic)) msg = latest_images_[topic];
        }

        if (!msg) {
            panels.push_back(ftxui::vbox({ ftxui::text(" Waiting: " + topic) | ftxui::center }) | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, char_h_per_img));
            continue;
        }

        auto c = ftxui::Canvas(target_w, target_h);
        
        // Encoding Detection
        bool is_bgr = false, is_mono8 = false, is_16uc1 = false, is_32fc1 = false;
        if (msg->encoding == "mono8") is_mono8 = true;
        else if (msg->encoding == "16UC1") is_16uc1 = true;
        else if (msg->encoding == "32FC1") is_32fc1 = true;
        else if (msg->encoding == "bgr8" || msg->encoding == "bgra8") is_bgr = true;
        else if (msg->encoding == "rgb8" || msg->encoding == "rgba8") { /* RGB handled by default */ }
        else is_bgr = true; // Fallback

        int channels = (msg->encoding.find("8") != std::string::npos && msg->encoding.find("a") != std::string::npos) ? 4 : 3;
        if (is_mono8) channels = 1;

        float sw = (float)msg->width / target_w, sh = (float)msg->height / target_h;
        for (int y = 0; y < target_h; ++y) {
            for (int x = 0; x < target_w; ++x) {
                int ix = (int)(x * sw), iy = (int)(y * sh);
                uint8_t r = 0, g = 0, b = 0;

                if (is_16uc1) {
                    size_t idx = (iy * msg->step) + (ix * 2);
                    if (idx + 1 < msg->data.size()) {
                        uint16_t depth; std::memcpy(&depth, &msg->data[idx], 2);
                        uint8_t val = (uint8_t)std::clamp((int)depth / 40, 0, 255); // Scale ~10m to 255
                        r = g = b = val;
                    }
                } else if (is_32fc1) {
                    size_t idx = (iy * msg->step) + (ix * 4);
                    if (idx + 3 < msg->data.size()) {
                        float depth; std::memcpy(&depth, &msg->data[idx], 4);
                        uint8_t val = (uint8_t)std::clamp((int)(depth * 25.0f), 0, 255); // Scale 10m to 255
                        r = g = b = val;
                    }
                } else if (is_mono8) {
                    size_t idx = (iy * msg->step) + ix;
                    if (idx < msg->data.size()) r = g = b = msg->data[idx];
                } else {
                    size_t idx = (iy * msg->step) + (ix * channels);
                    if (idx + 2 < msg->data.size()) {
                        if (is_bgr) { b = msg->data[idx]; g = msg->data[idx+1]; r = msg->data[idx+2]; }
                        else { r = msg->data[idx]; g = msg->data[idx+1]; b = msg->data[idx+2]; }
                    }
                }
                c.DrawPoint(x, y, true, ftxui::Color::RGB(r, g, b));
            }
        }

        panels.push_back(ftxui::vbox({
            ftxui::text(topic) | ftxui::bold | ftxui::color(ftxui::Color::Cyan) | ftxui::hcenter,
            ftxui::canvas(std::move(c)) | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, target_w / 2) | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, target_h / 4) | ftxui::hcenter
        }) | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, char_h_per_img));
    }

    return ftxui::vbox(std::move(panels));
}

} // namespace terminal_rviz
