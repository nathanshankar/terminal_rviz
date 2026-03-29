#ifndef TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_
#define TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_

#include <mutex>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"
#include "tf2_ros/buffer.h"
#include "terminal_rviz/display.hpp"

// Forward declarations for Assimp
struct aiScene;
struct aiNode;
struct aiMesh;
template <typename TReal> class aiMatrix4x4t;
typedef aiMatrix4x4t<float> aiMatrix4x4;

namespace terminal_rviz {

struct MeshPoint {
    tf2::Vector3 pos;
    ftxui::Color color;
};

struct Mesh {
    std::vector<MeshPoint> points; 
};

class RobotModelDisplay : public Display {
public:
    explicit RobotModelDisplay(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    void onInitialize() override;
    void render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) override;
    
    void setTopic(const std::string& topic) override;
    std::string getMessageType() const override { return "std_msgs/msg/String"; }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg);
    void render_link(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame);
    void render_geometry(urdf::GeometrySharedPtr geom, const tf2::Transform& tf, RvizRenderer& renderer, ftxui::Canvas& canvas, urdf::MaterialSharedPtr material);
    
    std::shared_ptr<Mesh> get_or_load_mesh(const std::string& uri);
    void process_assimp_node(const aiScene* scene, const aiNode* node, const aiMatrix4x4& transform, std::shared_ptr<Mesh> mesh_out);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    std::mutex mtx_;
    urdf::Model model_;
    bool model_loaded_ = false;

    std::map<std::string, std::shared_ptr<Mesh>> mesh_cache_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_DISPLAYS_ROBOT_MODEL_DISPLAY_HPP_
