#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

Display::Display(const std::string& name, rclcpp::Node::SharedPtr node)
    : name_(name), node_(node) {}

} // namespace terminal_rviz
