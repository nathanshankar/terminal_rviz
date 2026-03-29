#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "terminal_rviz/visualizer.hpp"
#include "terminal_rviz/displays/grid_display.hpp"
#include "terminal_rviz/displays/pointcloud_display.hpp"
#include "terminal_rviz/displays/marker_display.hpp"
#include "terminal_rviz/displays/tf_display.hpp"
#include "terminal_rviz/displays/laserscan_display.hpp"
#include "terminal_rviz/displays/map_display.hpp"
#include "terminal_rviz/displays/odometry_display.hpp"
#include "terminal_rviz/displays/image_display.hpp"
#include "terminal_rviz/displays/robot_model_display.hpp"
#include "terminal_rviz/displays/nav2_display.hpp"

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
    auto visualizer = std::make_shared<Visualizer>(node);

    // G: Grid (Handled separately)
    auto grid = std::make_shared<GridDisplay>(node);
    grid->onInitialize();
    visualizer->set_grid_display(grid);

    // 1: RobotModel
    auto robot_model = std::make_shared<RobotModelDisplay>(node, visualizer->get_tf_buffer());
    robot_model->onInitialize();
    visualizer->add_display(robot_model);

    // 2: TF
    auto tf = std::make_shared<TFDisplay>(node);
    tf->onInitialize();
    tf->setEnabled(false);
    visualizer->add_display(tf);

    // 3: PointCloud
    auto pc = std::make_shared<PointCloudDisplay>(node);
    pc->onInitialize();
    visualizer->add_display(pc);

    // 4: LaserScan
    auto scan = std::make_shared<LaserScanDisplay>(node);
    scan->onInitialize();
    visualizer->add_display(scan);

    // 5: Map
    auto map = std::make_shared<MapDisplay>(node);
    map->onInitialize();
    visualizer->add_display(map);

    // 6: Image
    auto img = std::make_shared<ImageDisplay>(node);
    img->onInitialize();
    visualizer->add_display(img);

    // 7: Marker
    auto markers = std::make_shared<MarkerDisplay>(node);
    markers->onInitialize();
    visualizer->add_display(markers);

    // 8: MarkerArray
    auto markers_array = std::make_shared<MarkerDisplay>(node);
    markers_array->setName("MarkerArray");
    markers_array->setPreferredType("visualization_msgs/msg/MarkerArray");
    markers_array->onInitialize();
    markers_array->setTopic("marker_array");
    visualizer->add_display(markers_array);

    // 9: Odometry
    auto odom = std::make_shared<OdometryDisplay>(node);
    odom->onInitialize();
    visualizer->add_display(odom);

    // 0: Nav2
    auto nav2 = std::make_shared<Nav2Display>(node);
    nav2->onInitialize();
    visualizer->add_display(nav2);

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
