#include "terminal_rviz/displays/image_display.hpp"

namespace terminal_rviz {

ImageDisplay::ImageDisplay(rclcpp::Node::SharedPtr node)
    : Display("Image", node) {}

void ImageDisplay::onInitialize() {
    topic_ = node_->declare_parameter(name_ + ".topic", "image_raw");
    setTopic(topic_);
}

void ImageDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        topic_, 10, std::bind(&ImageDisplay::callback, this, std::placeholders::_1));
}

void ImageDisplay::callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_msg_ = msg;
}

void ImageDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_) return;

    sensor_msgs::msg::Image::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    // Render image as a small overlay or in corner
    // For now, let's just project it to a plane in 3D or draw in 2D
    // Drawing in 2D (top-left)
    
    int img_w = msg->width;
    int img_h = msg->height;
    
    // Subsample to fit in a small corner
    int target_w = 40;
    int target_h = 30;
    
    float sw = (float)img_w / target_w;
    float sh = (float)img_h / target_h;

    for (int y = 0; y < target_h; ++y) {
        for (int x = 0; x < target_w; ++x) {
            int ix = (int)(x * sw);
            int iy = (int)(y * sh);
            
            size_t idx = (iy * msg->step) + (ix * 3); // Assuming RGB8
            if (idx + 2 < msg->data.size()) {
                uint8_t r = msg->data[idx];
                uint8_t g = msg->data[idx+1];
                uint8_t b = msg->data[idx+2];
                canvas.DrawPoint(x, y, true, ftxui::Color::RGB(r, g, b));
            }
        }
    }
}

} // namespace terminal_rviz
