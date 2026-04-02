cmake_minimum_required(VERSION 3.8)
project(terminal_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(assimp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

option(WITH_GPU "Build with OpenCL GPU support" ON)

if(WITH_GPU)
  find_package(OpenCL)
  if(OpenCL_FOUND)
    message(STATUS "OpenCL found: ${OpenCL_LIBRARIES}")
    add_definitions(-DUSE_GPU)
  else()
    message(WARNING "OpenCL NOT found, building without GPU support")
    set(WITH_GPU OFF)
  endif()
endif()

# Fetch FTXUI
include(FetchContent)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v5.0.0
)
FetchContent_MakeAvailable(ftxui)

# Create a library for the core visualizer/renderer
add_library(${PROJECT_NAME}_core
  src/config_helper.cpp
  src/display.cpp
  src/displays/accel_stamped_display.cpp
  src/displays/axes_display.cpp
  src/displays/camera_info_display.cpp
  src/displays/effort_display.cpp
  src/displays/fluid_pressure_display.cpp
  src/displays/grid_cells_display.cpp
  src/displays/grid_display.cpp
  src/displays/illuminance_display.cpp
  src/displays/image_display.cpp
  src/displays/laserscan_display.cpp
  src/displays/legacy_pointcloud_display.cpp
  src/displays/map_display.cpp
  src/displays/marker_display.cpp
  src/displays/nav2_display.cpp
  src/displays/odometry_display.cpp
  src/displays/path_display.cpp
  src/displays/point_stamped_display.cpp
  src/displays/pointcloud_display.cpp
  src/displays/polygon_display.cpp
  src/displays/pose_array_display.cpp
  src/displays/pose_display.cpp
  src/displays/pose_with_covariance_display.cpp
  src/displays/range_display.cpp
  src/displays/relative_humidity_display.cpp
  src/displays/robot_model_display.cpp
  src/displays/temperature_display.cpp
  src/displays/tf_display.cpp
  src/displays/twist_stamped_display.cpp
  src/displays/wrench_display.cpp
  src/renderer.cpp
  src/gpu_renderer.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ASSIMP_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  urdf
  urdfdom_headers
  ament_index_cpp
  std_msgs
  nav2_msgs
  action_msgs
  actionlib_msgs
)

target_link_libraries(${PROJECT_NAME}_core
  ftxui::screen
  ftxui::dom
  ftxui::component
  ${ASSIMP_LIBRARIES}
)
if(WITH_GPU AND OpenCL_FOUND)
  target_link_libraries(${PROJECT_NAME}_core OpenCL::OpenCL)
endif()

# Main executable
add_executable(terminal_rviz_node
  src/terminal_rviz_node.cpp
  src/visualizer.cpp
)

target_include_directories(terminal_rviz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(terminal_rviz_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  urdf
  urdfdom_headers
  ament_index_cpp
  std_msgs
  nav2_msgs
  action_msgs
  actionlib_msgs
)

target_link_libraries(terminal_rviz_node
  ${PROJECT_NAME}_core
  ${ASSIMP_LIBRARIES}
)

install(TARGETS 
  ${PROJECT_NAME}_core
  terminal_rviz_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
