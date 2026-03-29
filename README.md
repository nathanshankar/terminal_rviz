# Terminal RViz

Terminal RViz is a lightweight, high-performance 3D visualizer for ROS 2 that runs entirely within a terminal environment. Built using FTXUI and a custom Braille-based software renderer, it provides a feature-rich alternative to standard RViz for headless systems, remote SSH sessions, or resource-constrained environments.

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

## Prerequisites

- **ROS 2**
- **Dependencies**:
    - `rclcpp`, `rclcpp_action`
    - `sensor_msgs`, `nav_msgs`, `nav2_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`
    - `tf2`, `tf2_ros`, `tf2_geometry_msgs`
    - `urdf`, `assimp`, `ament_index_cpp`
    - `ftxui` (Library handled via CMake FetchContent or system install)

## Build Instructions

### General Build
```bash
cd ~/your_ws
colcon build --packages-select terminal_rviz --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

Run the node with optional remappings for namespaced robots, if you are using one:

```bash
ros2 run terminal_rviz terminal_rviz_node --ros-args \
  -r __ns:=/{namespace} \
  -r tf:=/{namespace}/tf \
  -r tf_static:=/{namespace}/tf_static
```

## Controls

### Keyboard Shortcuts
- **0 - 9**: Toggle standard displays (RobotModel, TF, PointCloud, etc.).
- **0**: Toggle Nav2 Status Dashboard.
- **G**: Toggle XY Grid.
- **V**: Cycle Mouse Tool (NAV2 -> ORBIT -> PAN).
- **R**: Reset View 
- **M**: Top-Down View 
- **Tab**: Cycle plugin focus in the settings sidebar.
- **T / Y**: Navigate through available Topics or TF Frames.
- **Space**: Contextual Toggle (Enable/Disable specific Topics or Frames).
- **Q / Esc**: Quit application.

### Nav2 Command Shortcuts 
- **Enter**: Confirm and Send built waypoint queue to Nav2.
- **Backspace**: Remove the last added waypoint from the queue.
- **C**: Immediate Navigation Cancel and queue clear.

### Mouse Interaction (Touchpad Optimized)
- **Tool: NAV2**: Click + Drag to place a 3D waypoint arrow.
- **Tool: ORBIT**: Click + Drag to rotate the camera.
- **Tool: PAN**: Click + Drag to slide the view (Window-Relative).
- **Scroll Wheel**: Zoom In / Out.

## Performance Notes
To achieve maximum frame rates, ensure you build with `-DCMAKE_BUILD_TYPE=Release`. The visualizer automatically caps its internal update rate to 20Hz to balance CPU usage with UI responsiveness.
