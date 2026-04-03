#ifndef TERMINAL_RVIZ_DISPLAYS_GRID_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_GRID_DISPLAY_HPP_

#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class GridDisplay : public Display {
public:
    explicit GridDisplay(rclcpp::Node::SharedPtr node);

    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    std::string getMessageType() const override { return "None"; }

private:
    int cell_count_ = 10;
    float cell_size_ = 1.0f;
    ftxui::Color color_ = ftxui::Color::RGB(160, 160, 160);
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_GRID_DISPLAY_HPP_
