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

void ImageDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    sensor_msgs::msg::Image::SharedPtr msg;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = current_msg_;
    }

    if (!msg) return;

    int img_w = msg->width, img_h = msg->height;
    int target_w = 40, target_h = 30;
    float sw = (float)img_w / target_w, sh = (float)img_h / target_h;

    for (int y = 0; y < target_h; ++y) {
        for (int x = 0; x < target_w; ++x) {
            int ix = (int)(x * sw), iy = (int)(y * sh);
            size_t idx = (iy * msg->step) + (ix * 3);
            if (idx + 2 < msg->data.size()) {
                renderer.plot(x, y, -10.0f, ftxui::Color::RGB(msg->data[idx], msg->data[idx+1], msg->data[idx+2]));
            }
        }
    }
}

} // namespace terminal_rviz
