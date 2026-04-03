# TViz

Terminal RViz is a lightweight, high-performance 3D visualizer for ROS 2 that runs entirely within a terminal environment. Built using FTXUI and a custom Braille-based software renderer, it provides a feature-rich alternative to standard RViz for headless systems, remote SSH sessions, or resource-constrained environments.

<img width="1842" height="951" alt="image" src="https://github.com/user-attachments/assets/d30913ef-d802-4d93-b3ee-ce58a6a58728" />

## Build Status

| Distribution | Status |
| :--- | :--- |
| **ROS 2 Foxy** | [![Foxy Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/terminal_rviz/ros_build_foxy.yml?branch=main&label=Build)](https://github.com/nathanshankar/terminal_rviz/actions/workflows/ros_build_foxy.yml) |
| **ROS 2 Humble** | [![Humble Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/terminal_rviz/ros_build_humble.yml?branch=main&label=Build)](https://github.com/nathanshankar/terminal_rviz/actions/workflows/ros_build_humble.yml) |
| **ROS 2 Iron** | [![Iron Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/terminal_rviz/ros_build_iron.yml?branch=main&label=Build)](https://github.com/nathanshankar/terminal_rviz/actions/workflows/ros_build_iron.yml) |
| **ROS 2 Jazzy** | [![Jazzy Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/terminal_rviz/ros_build_jazzy.yml?branch=main&label=Build)](https://github.com/nathanshankar/terminal_rviz/actions/workflows/ros_build_jazzy.yml) |


## Features

- **High-Density 3D Rendering**: Utilizes Braille characters (2x4 dot matrix) with dot-level Z-buffering and character-level color management.
- **10 Integrated Plugins**: 
    - **RobotModel**: Unified URDF loading with Assimp-based mesh sampling (DAE/STL).
    - **TF/TF2**: Dynamic transform tree visualization with per-frame toggles.
    - **Nav2 Dashboard**: Full Action Client integration for "Navigate Through Poses" with waypoint queuing.
    - **Image Visualizer**: Support for up to 2 simultaneous feeds with auto-normalization for Depth (16UC1/32FC1) and Mono streams.
    - **Map**: High-contrast Occupancy Grid rendering (Magenta/White scheme).
    - **PointCloud2 & LaserScan**: Efficient spatial data visualization with intensity/RGB support.
    - **Marker & MarkerArray**: Full support for 3D primitives (Cubes, Spheres, Cylinders, Arrows, Lines).
    - **Odometry**: Directional movement history tracking (last 10 poses).
- **Optimized Pipeline**: 2D fast-path line drawing, dirty-cell buffer tracking, and adaptive robot model density for smooth performance.

## Development Branch (devel) Changes

The `devel` branch includes several major updates and new features that are being prepared for the next release. Here's a checklist of the key changes:
- [x] **Click Support**: Most of the panels support mouse click.
- [x] **Expanded Plugin Suite**: Added over 20 new display types including `AccelStamped`, `Effort`, `Path`, `PoseArray`, `Range`, `Temperature`, and `Wrench`.
- [x] **GPU-Accelerated Rendering**: Experimental support for GPU-based point cloud rendering using OpenCL.
- [x] **ROS 2 Bag Management**: Integrated `RosbagDisplay` for recording and managing playback directly from the terminal.
- [x] **Interactive Teleop**: New `TeleopDisplay` for keyboard-based robot control.
- [x] **Configuration Persistence**: Save and load visualizer setups using a custom `.nathan` format.
- [x] **Enhanced UI/UX**:
    - Sidebar hovering and right-click to disable/enable plugins.
    - Improved modal system for plugin, panel, and topic discovery.
    - Advanced mouse interaction including click-and-drag configuration adjustment.
- [x] **Advanced Rendering Improvements**:
    - Character-level color management and pixel averaging.
    - Performance-optimized 3D-to-2D projection pipeline with a dedicated `Projector` class.
- [x] **Improved Topic Discovery**: Automated discovery and categorization of available topics by message type.

## Prerequisites

- **ROS 2**
- **Dependencies**:
    - `rclcpp`, `rclcpp_action`
    - `sensor_msgs`, `nav_msgs`, `nav2_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`
    - `tf2`, `tf2_ros`, `tf2_geometry_msgs`
    - `urdf`, `assimp`, `ament_index_cpp`
    - `ftxui` (Library handled via CMake FetchContent or system install)

## Build Instructions

### Without GPU (default)
```bash
cd ~/your_ws
colcon build --packages-select terminal_rviz --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_GPU=OFF
source install/setup.bash
```

### With GPU (OpenCL-accelerated)
```bash
# With GPU (OpenCL-accelerated)
cd ~/your_ws
colcon build --packages-select terminal_rviz --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_GPU=ON
source install/setup.bash
```

## Usage

Run the node with optional remappings for namespaced robots, if you are using one:

### Without GPU (default)
```bash
ros2 run terminal_rviz terminal_rviz_node --ros-args \
  -r __ns:=/j100_0218 \
  -r tf:=/j100_0218/tf \
  -r tf_static:=/j100_0218/tf_static
```

### With GPU (OpenCL-accelerated)
```bash
ros2 run terminal_rviz terminal_rviz_node --ros-args \
  -r __ns:=/j100_0218 \
  -r tf:=/j100_0218/tf \
  -r tf_static:=/j100_0218/tf_static \
  -p use_gpu:=true
```

## Documentation

- **[Plugin Development Guide](PLUGIN_GUIDE.md)**: A comprehensive guide on how to create, implement, and register new display plugins for TViz.

## Controls

### Keyboard Shortcuts
- **P**: Open Add/Manage Plugins modal.
- **F**: Open Fixed Frame selection modal.
- **G**: Toggle XY Grid.
- **V**: Cycle Mouse Tool (NAV2 -> ORBIT -> PAN).
- **R**: Reset View to default perspective.
- **T**: Top-Down View (automatically centers on Map if available).
- **Tab**: Cycle plugin focus in the settings sidebar.
- **Y / H**: Navigate through available Topics or TF Frames for the selected plugin.
- **Space**: Contextual Toggle (Enable/Disable specific Topics or Frames).
- **+ / -**: Zoom In / Out.
- **Arrow Keys**: Pan Camera (X/Y) or move Z-axis.
- **W / A / S / D**: Rotate Camera (Pitch/Yaw).
- **Esc**: Quit application.

### Nav2 Command Shortcuts 
- **Enter**: Confirm and Send built waypoint queue to Nav2.
- **Backspace**: Remove the last added waypoint from the queue.
- **C**: Immediate Navigation Cancel and queue clear.

### Mouse Interaction (Touchpad Optimized)
- **Tool: NAV2**: Click + Drag to place a 3D waypoint arrow.
- **Tool: ORBIT**: Click + Drag to rotate the camera.
- **Tool: PAN**: Click + Drag to slide the view (Window-Relative).
- **Scroll Wheel**: Zoom In / Out.
- **Right Click (on sidebar)**: Toggle plugin enable/disable.
- **Left Click (on 2D UI)**: Interact with sliders, buttons, and topic selections.

## Performance Notes
To achieve maximum frame rates, ensure you build with `-DCMAKE_BUILD_TYPE=Release`. The visualizer automatically caps its internal update rate to 20Hz to balance CPU usage with UI responsiveness.
