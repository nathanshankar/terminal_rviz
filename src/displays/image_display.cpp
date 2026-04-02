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
    int max_image_rows = char_h_per_img - 3;
    int max_image_cols = 60; // Standard right panel width minus padding
    if (terminal.dimx < 120) max_image_cols = terminal.dimx / 3 - 4;

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

        // Calculate best fit aspect ratio
        float aspect = (float)msg->width / (float)msg->height;
        // Terminal characters are ~2x taller than wide, Braille is 2x4 dots. 
        // Dot-level aspect ratio: a Braille dot is approx square.
        int target_w, target_h;
        if (aspect > (float)max_image_cols / (max_image_rows * 0.5f)) {
            target_w = max_image_cols * 2;
            target_h = (int)(target_w / aspect);
        } else {
            target_h = max_image_rows * 4;
            target_w = (int)(target_h * aspect);
        }
        
        target_w = std::max(4, (target_w / 2) * 2);
        target_h = std::max(4, (target_h / 4) * 4);

        bool is_bgr = false, is_mono8 = false, is_16uc1 = false, is_32fc1 = false;
        if (msg->encoding == "mono8") is_mono8 = true;
        else if (msg->encoding == "16UC1") is_16uc1 = true;
        else if (msg->encoding == "32FC1") is_32fc1 = true;
        else if (msg->encoding == "bgr8" || msg->encoding == "bgra8") is_bgr = true;
        else if (msg->encoding == "rgb8" || msg->encoding == "rgba8") { /* Default */ }
        else is_bgr = true; 

        int channels = (msg->encoding.find("8") != std::string::npos && msg->encoding.find("a") != std::string::npos) ? 4 : 3;
        if (is_mono8) channels = 1;

        auto get_pixel = [&](int px, int py, uint8_t& r, uint8_t& g, uint8_t& b) {
            px = std::clamp(px, 0, (int)msg->width - 1);
            py = std::clamp(py, 0, (int)msg->height - 1);
            if (is_16uc1) {
                size_t idx = (py * msg->step) + (px * 2);
                uint16_t depth; std::memcpy(&depth, &msg->data[idx], 2);
                r = g = b = (uint8_t)std::clamp((int)depth / 40, 0, 255);
            } else if (is_32fc1) {
                size_t idx = (py * msg->step) + (px * 4);
                float depth; std::memcpy(&depth, &msg->data[idx], 4);
                r = g = b = (uint8_t)std::clamp((int)(depth * 25.0f), 0, 255);
            } else if (is_mono8) {
                r = g = b = msg->data[(py * msg->step) + px];
            } else {
                size_t idx = (py * msg->step) + (px * channels);
                if (is_bgr) { b = msg->data[idx]; g = msg->data[idx+1]; r = msg->data[idx+2]; }
                else { r = msg->data[idx]; g = msg->data[idx+1]; b = msg->data[idx+2]; }
            }
        };

        auto c = ftxui::Canvas(target_w, target_h);
        float sw = (float)msg->width / target_w, sh = (float)msg->height / target_h;

        for (int y = 0; y < target_h; ++y) {
            for (int x = 0; x < target_w; ++x) {
                float r_sum = 0, g_sum = 0, b_sum = 0;
                int count = 0;
                
                int sx_start = (int)(x * sw);
                int sx_end = (int)((x + 1) * sw);
                int sy_start = (int)(y * sh);
                int sy_end = (int)((y + 1) * sh);
                
                sx_end = std::max(sx_end, sx_start + 1);
                sy_end = std::max(sy_end, sy_start + 1);

                for (int sy = sy_start; sy < sy_end; ++sy) {
                    for (int sx = sx_start; sx < sx_end; ++sx) {
                        uint8_t pr, pg, pb;
                        get_pixel(sx, sy, pr, pg, pb);
                        r_sum += pr; g_sum += pg; b_sum += pb;
                        count++;
                    }
                }
                
                uint8_t fr = (uint8_t)(r_sum / count);
                uint8_t fg = (uint8_t)(g_sum / count);
                uint8_t fb = (uint8_t)(b_sum / count);

                // Contrast/Gamma boost for terminal
                auto boost = [](uint8_t v) {
                    float f = v / 255.0f;
                    f = std::pow(f, 0.85f); // Brighten midtones
                    return (uint8_t)std::clamp(f * 255.0f, 0.0f, 255.0f);
                };

                c.DrawPoint(x, y, true, ftxui::Color::RGB(boost(fr), boost(fg), boost(fb)));
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
