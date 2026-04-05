#ifndef TERMINAL_RVIZ_ROBOT_RENDERER_HPP_
#define TERMINAL_RVIZ_ROBOT_RENDERER_HPP_

#include <mutex>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include "urdf/model.h"
#include "tf2_ros/buffer.h"
#include "terminal_rviz/renderer.hpp"

// Forward declarations for Assimp
struct aiScene;
struct aiNode;
struct aiMesh;
template <typename TReal> class aiMatrix4x4t;
typedef aiMatrix4x4t<float> aiMatrix4x4;

namespace terminal_rviz {

struct MeshPoint {
    tf2::Vector3 pos;
    uint8_t r, g, b;
};

struct Mesh {
    std::vector<MeshPoint> points; 
};

class RobotRenderer {
public:
    RobotRenderer();
    
    bool init_urdf(const std::string& urdf_xml);
    bool is_loaded() const { return model_loaded_; }

    void render(RvizRenderer& renderer, const std::string& fixed_frame, 
                std::shared_ptr<tf2_ros::Buffer> tf_buffer, float global_alpha = 1.0f);
                
    void render_state(RvizRenderer& renderer, const std::string& fixed_frame, 
                      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                      const std::map<std::string, double>& joint_positions, float global_alpha = 1.0f);

private:
    void render_link(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, 
                     const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer, float alpha);
                     
    void render_link_recursive(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, 
                               const tf2::Transform& parent_tf, 
                               const std::map<std::string, double>& joint_positions, float alpha);

    void render_geometry(urdf::GeometrySharedPtr geom, const tf2::Transform& tf, 
                         RvizRenderer& renderer, urdf::MaterialSharedPtr material, float alpha);
    
    std::shared_ptr<Mesh> get_or_load_mesh(const std::string& uri);
    void process_assimp_node(const aiScene* scene, const aiNode* node, const aiMatrix4x4& transform, std::shared_ptr<Mesh> mesh_out);

    std::mutex mtx_;
    urdf::Model model_;
    bool model_loaded_ = false;
    std::map<std::string, std::shared_ptr<Mesh>> mesh_cache_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_ROBOT_RENDERER_HPP_
