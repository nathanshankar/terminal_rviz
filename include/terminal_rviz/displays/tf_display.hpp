#ifndef TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_

#include <mutex>
#include <map>
#include <string>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class TFDisplay : public Display {
public:
    explicit TFDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    std::string getMessageType() const override { return "TF"; }

    std::vector<std::string> getDiscoveredFrames();
    void toggleFrame(const std::string& frame);
    bool isFrameEnabled(const std::string& frame) const;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::set<std::string> enabled_frames_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_TF_DISPLAY_HPP_
