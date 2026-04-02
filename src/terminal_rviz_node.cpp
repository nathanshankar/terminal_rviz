#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/displays/accel_stamped_display.hpp"
#include "terminal_rviz/displays/axes_display.hpp"
#include "terminal_rviz/displays/camera_info_display.hpp"
#include "terminal_rviz/displays/effort_display.hpp"
#include "terminal_rviz/displays/fluid_pressure_display.hpp"
#include "terminal_rviz/displays/grid_cells_display.hpp"
#include "terminal_rviz/displays/grid_display.hpp"
#include "terminal_rviz/displays/illuminance_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"
#include "terminal_rviz/displays/laserscan_display.hpp"
#include "terminal_rviz/displays/legacy_pointcloud_display.hpp"
#include "terminal_rviz/displays/map_display.hpp"
#include "terminal_rviz/displays/marker_display.hpp"
#include "terminal_rviz/displays/nav2_display.hpp"
#include "terminal_rviz/displays/odometry_display.hpp"
#include "terminal_rviz/displays/path_display.hpp"
#include "terminal_rviz/displays/point_stamped_display.hpp"
#include "terminal_rviz/displays/pointcloud_display.hpp"
#include "terminal_rviz/displays/polygon_display.hpp"
#include "terminal_rviz/displays/pose_array_display.hpp"
#include "terminal_rviz/displays/pose_display.hpp"
#include "terminal_rviz/displays/pose_with_covariance_display.hpp"
#include "terminal_rviz/displays/range_display.hpp"
#include "terminal_rviz/displays/relative_humidity_display.hpp"
#include "terminal_rviz/displays/robot_model_display.hpp"
#include "terminal_rviz/displays/temperature_display.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/twist_stamped_display.hpp"
#include "terminal_rviz/displays/wrench_display.hpp"

using namespace terminal_rviz;

class DummyDisplay : public Display {
public:
    DummyDisplay(const std::string& name, rclcpp::Node::SharedPtr node, const std::string& type = "None") 
        : Display(name, node), type_(type) {}
    void render(RvizRenderer&, ftxui::Canvas&, const std::string&, std::shared_ptr<tf2_ros::Buffer>) override {}
    std::string getMessageType() const override { return type_; }
private:
    std::string type_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("terminal_rviz");
    node->declare_parameter("use_gpu", false);
    bool use_gpu = node->get_parameter("use_gpu").as_bool();

    auto visualizer = std::make_shared<Visualizer>(node);
    visualizer->set_use_gpu(use_gpu);

    // AccelStamped
    auto accel = std::make_shared<AccelStampedDisplay>(node);
    accel->onInitialize();
    visualizer->add_display(accel);

    // Axes
    auto axes = std::make_shared<AxesDisplay>(node);
    axes->onInitialize();
    visualizer->add_display(axes);

    // CameraInfo
    auto camera_info = std::make_shared<CameraInfoDisplay>(node);
    camera_info->onInitialize();
    visualizer->add_display(camera_info);

    // Effort
    auto effort = std::make_shared<EffortDisplay>(node);
    effort->onInitialize();
    visualizer->add_display(effort);

    // FluidPressure
    auto fluid_pressure = std::make_shared<FluidPressureDisplay>(node);
    fluid_pressure->onInitialize();
    visualizer->add_display(fluid_pressure);

    // Grid (Handled separately)
    auto grid = std::make_shared<GridDisplay>(node);
    grid->onInitialize();
    visualizer->set_grid_display(grid);

    // GridCells
    auto grid_cells = std::make_shared<GridCellsDisplay>(node);
    grid_cells->onInitialize();
    visualizer->add_display(grid_cells);

    // Illuminance
    auto illuminance = std::make_shared<IlluminanceDisplay>(node);
    illuminance->onInitialize();
    visualizer->add_display(illuminance);

    // Image
    auto img = std::make_shared<ImageDisplay>(node);
    img->onInitialize();
    visualizer->add_display(img);

    // LaserScan
    auto scan = std::make_shared<LaserScanDisplay>(node);
    scan->onInitialize();
    visualizer->add_display(scan);

    // Map
    auto map = std::make_shared<MapDisplay>(node);
    map->onInitialize();
    visualizer->add_display(map);

    // Marker
    auto marker = std::make_shared<MarkerDisplay>(node);
    marker->onInitialize();
    visualizer->add_display(marker);

    // MarkerArray (Using MarkerDisplay but named "MarkerArray")
    auto marker_array = std::make_shared<MarkerDisplay>(node);
    marker_array->setName("MarkerArray");
    marker_array->setPreferredType("visualization_msgs/msg/MarkerArray");
    marker_array->onInitialize();
    visualizer->add_display(marker_array);

    // Nav2
    auto nav2 = std::make_shared<Nav2Display>(node);
    nav2->onInitialize();
    visualizer->add_display(nav2);

    // Odometry
    auto odom = std::make_shared<OdometryDisplay>(node);
    odom->onInitialize();
    visualizer->add_display(odom);

    // Path
    auto path = std::make_shared<PathDisplay>(node);
    path->onInitialize();
    visualizer->add_display(path);

    // PointCloud (Legacy)
    auto legacy_pc = std::make_shared<LegacyPointCloudDisplay>(node);
    legacy_pc->onInitialize();
    visualizer->add_display(legacy_pc);

    // PointCloud2
    auto pc2 = std::make_shared<PointCloudDisplay>(node);
    pc2->onInitialize();
    visualizer->add_display(pc2);

    // PointStamped
    auto point_stamped = std::make_shared<PointStampedDisplay>(node);
    point_stamped->onInitialize();
    visualizer->add_display(point_stamped);

    // Polygon
    auto polygon = std::make_shared<PolygonDisplay>(node);
    polygon->onInitialize();
    visualizer->add_display(polygon);

    // Pose
    auto pose = std::make_shared<PoseDisplay>(node);
    pose->onInitialize();
    visualizer->add_display(pose);

    // PoseArray
    auto pose_array = std::make_shared<PoseArrayDisplay>(node);
    pose_array->onInitialize();
    visualizer->add_display(pose_array);

    // PoseWithCovariance
    auto pose_with_covariance = std::make_shared<PoseWithCovarianceDisplay>(node);
    pose_with_covariance->onInitialize();
    visualizer->add_display(pose_with_covariance);

    // Range
    auto range = std::make_shared<RangeDisplay>(node);
    range->onInitialize();
    visualizer->add_display(range);

    // RelativeHumidity
    auto humidity = std::make_shared<RelativeHumidityDisplay>(node);
    humidity->onInitialize();
    visualizer->add_display(humidity);

    // RobotModel
    auto robot_model = std::make_shared<RobotModelDisplay>(node, visualizer->get_tf_buffer());
    robot_model->onInitialize();
    visualizer->add_display(robot_model);

    // Temperature
    auto temp = std::make_shared<TemperatureDisplay>(node);
    temp->onInitialize();
    visualizer->add_display(temp);

    // TF
    auto tf = std::make_shared<TFDisplay>(node);
    tf->onInitialize();
    tf->setEnabled(false);
    visualizer->add_display(tf);

    // TwistStamped
    auto twist_stamped = std::make_shared<TwistStampedDisplay>(node);
    twist_stamped->onInitialize();
    visualizer->add_display(twist_stamped);

    // Wrench
    auto wrench = std::make_shared<WrenchDisplay>(node);
    wrench->onInitialize();
    visualizer->add_display(wrench);

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    visualizer->run();

    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return 0;
}
