#include "terminal_rviz/displays/grid_display.hpp"

namespace terminal_rviz {

GridDisplay::GridDisplay(rclcpp::Node::SharedPtr node)
    : Display("Grid", node) {}

void GridDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_) return;

    float half_size = (cell_count_ * cell_size_) / 2.0f;

    for (int i = 0; i <= cell_count_; ++i) {
        float offset = -half_size + i * cell_size_;
        renderer.draw_line(offset, -half_size, 0.0f, offset, half_size, 0.0f, color_);
        renderer.draw_line(-half_size, offset, 0.0f, half_size, offset, 0.0f, color_);
    }
}

} // namespace terminal_rviz
