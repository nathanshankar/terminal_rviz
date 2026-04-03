#ifndef TERMINAL_RVIZ_DISPLAYS_GRID_CELLS_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_GRID_CELLS_DISPLAY_HPP_

#include <mutex>
#include <map>
#include <vector>

#include "nav_msgs/msg/grid_cells.hpp"
#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

class GridCellsDisplay : public Display {
public:
    explicit GridCellsDisplay(rclcpp::Node::SharedPtr node);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    ftxui::Element render_2d(bool nav2_active = false, int config_scroll = 0) override;
    bool handle_event(ftxui::Event event, int scroll_offset = 0) override;
    void setTopic(const std::string& topic) override;
    bool isTopicEnabled(const std::string& topic) const override;
    std::vector<std::string> getEnabledTopics() const override;
    
    TopicConfig getTopicConfig(const std::string& topic) override;
    void setTopicConfig(const std::string& topic, const TopicConfig& config) override;

    std::string getMessageType() const override { return "nav_msgs/msg/GridCells"; }

private:
    void callback(const nav_msgs::msg::GridCells::SharedPtr msg, const std::string& topic);

    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr> subs_;
    std::map<std::string, nav_msgs::msg::GridCells::SharedPtr> latest_msgs_;
    std::map<std::string, TopicConfig> configs_;
    std::vector<std::string> enabled_topics_;
    
    mutable std::mutex mtx_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_GRID_CELLS_DISPLAY_HPP_
