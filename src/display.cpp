#include "terminal_rviz/display.hpp"

namespace terminal_rviz {

Display::Display(const std::string& name, rclcpp::Node::SharedPtr node)
    : name_(name), node_(node) {}

void Display::render_styled_point(RvizRenderer& renderer, float x, float y, float z, const TopicConfig& cfg, uint8_t r, uint8_t g, uint8_t b) {
    if (cfg.style == "Points") {
        renderer.draw_point(x, y, z, r, g, b, cfg.alpha);
    } else if (cfg.style == "Squares" || cfg.style == "Flat Squares" || cfg.style == "Tiles") {
        renderer.draw_tile(x, y, z, cfg.size, cfg.size, r, g, b, cfg.alpha);
    } else if (cfg.style == "Spheres") {
        renderer.draw_sphere(x, y, z, cfg.size * 0.5f, r, g, b, cfg.alpha);
    } else if (cfg.style == "Boxes") {
        renderer.draw_box(x, y, z, cfg.size, cfg.size, cfg.size, r, g, b, cfg.alpha);
    } else {
        renderer.draw_point(x, y, z, r, g, b, cfg.alpha);
    }
}

} // namespace terminal_rviz
