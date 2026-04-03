# TViz Plugin Development Guide

This guide outlines the architecture of TViz plugins and provides step-by-step instructions on how to implement and integrate a new display plugin.

## Plugin Architecture

All TViz plugins must inherit from the `terminal_rviz::Display` base class. This class provides the standard interface for ROS 2 interaction, rendering, and UI management.

### The `Display` Base Class

Key virtual methods to override:

- `onInitialize()`: Called once when the plugin is created. Use this for one-time setup that doesn't require a topic.
- `setTopic(const std::string& topic)`: Called when a user selects a topic for the plugin. Handle subscription logic here.
- `render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer)`: The primary 3D rendering loop. Use the `renderer` to draw points, lines, and shapes.
- `render_2d(bool nav2_active, int config_scroll)`: Optional. Renders a 2D UI overlay (e.g., settings or data summary) in the right-hand panel.
- `handle_event(ftxui::Event event, int scroll_offset)`: Optional. Handles keyboard and mouse events specifically for this plugin's 2D UI.
- `getMessageType()`: **Required**. Returns the ROS 2 message type string (e.g., `"sensor_msgs/msg/PointCloud2"`).

## Implementation Steps

### 1. Create the Header (`include/terminal_rviz/displays/my_plugin_display.hpp`)

Define your class, inheriting from `Display`. Include a mutex for thread-safe access to latest messages.

```cpp
#ifndef TERMINAL_RVIZ_DISPLAYS_MY_PLUGIN_HPP_
#define TERMINAL_RVIZ_DISPLAYS_MY_PLUGIN_HPP_

#include "terminal_rviz/display.hpp"
#include "my_msgs/msg/my_type.hpp"
#include <mutex>

namespace terminal_rviz {

class MyPluginDisplay : public Display {
public:
    explicit MyPluginDisplay(rclcpp::Node::SharedPtr node);

    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, 
                const std::string& fixed_frame, 
                std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "my_msgs/msg/MyType"; }

private:
    rclcpp::Subscription<my_msgs::msg::MyType>::SharedPtr sub_;
    my_msgs::msg::MyType::SharedPtr latest_msg_;
    mutable std::mutex mtx_;
};

}
#endif
```

### 2. Implement the Source (`src/displays/my_plugin_display.cpp`)

#### Handling Topics
TViz allows multiple topics per plugin type. `setTopic` is usually used to toggle or set the active subscription.

```cpp
void MyPluginDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (topic_ == topic) {
        sub_.reset();
        topic_ = "";
        return;
    }
    
    topic_ = topic;
    sub_ = node_->create_subscription<my_msgs::msg::MyType>(
        topic, 10, [this](const my_msgs::msg::MyType::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_msg_ = msg;
        });
}
```

#### Rendering
Transform the message coordinates to the `fixed_frame` using the provided `tf_buffer`, then use the `renderer` to draw.

```cpp
void MyPluginDisplay::render(RvizRenderer& renderer, ftxui::Canvas&, 
                             const std::string& fixed_frame, 
                             std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    if (!enabled_ || !latest_msg_) return;

    std::lock_guard<std::mutex> lock(mtx_);
    
    // 1. Get Transform
    tf2::Transform tf;
    try {
        auto t = tf_buffer->lookupTransform(fixed_frame, latest_msg_->header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(t.transform, tf);
    } catch (...) { return; }

    // 2. Transform Data
    tf2::Vector3 p = tf * tf2::Vector3(latest_msg_->x, latest_msg_->y, latest_msg_->z);

    // 3. Draw
    renderer.draw_sphere(p.x(), p.y(), p.z(), 0.1f, 255, 255, 0); // Yellow sphere
}
```

### 3. Register the Plugin

To make your plugin available in the application:

1.  **Add to `CMakeLists.txt`**: Add the `.cpp` file to the `add_executable` source list.
2.  **Add to `terminal_rviz_node.cpp`**:
    - Include the header.
    - Instantiate the plugin.
    - Call `onInitialize()`.
    - Add it to the visualizer using `visualizer->add_display(my_plugin)`.

```cpp
// src/terminal_rviz_node.cpp
#include "terminal_rviz/displays/my_plugin_display.hpp"

// ... inside main ...
auto my_plugin = std::make_shared<MyPluginDisplay>(node);
my_plugin->onInitialize();
visualizer->add_display(my_plugin);
```

## Advanced Features

### 2D Settings Panel
Use `ConfigHelper` to render standard summaries or create custom FTXUI elements in `render_2d`.

### Custom Events
Override `handle_event` to capture mouse clicks or key presses when your plugin is focused (selected in the sidebar).

### Multi-Topic Support
If your plugin supports multiple simultaneous topics (like `PointCloud2`), maintain a `std::map<std::string, Subscription>` and a `std::vector<std::string> enabled_topics_`.
