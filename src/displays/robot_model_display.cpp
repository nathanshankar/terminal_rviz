#include "terminal_rviz/displays/robot_model_display.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <urdf/model.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>

namespace terminal_rviz {

RobotModelDisplay::RobotModelDisplay(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : Display("RobotModel", node), tf_buffer_(tf_buffer) {
    config_.alpha = 1.0f;
}

void RobotModelDisplay::onInitialize() {
    setTopic("/robot_description");
}

void RobotModelDisplay::setTopic(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mtx_);
    topic_ = topic;
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local(),
        std::bind(&RobotModelDisplay::callback, this, std::placeholders::_1));
}

void RobotModelDisplay::callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (model_.initString(msg->data)) {
        model_loaded_ = true;
        mesh_cache_.clear();
        RCLCPP_INFO(node_->get_logger(), "RobotModel: Successfully parsed URDF");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "RobotModel: Failed to parse URDF");
    }
}

void RobotModelDisplay::render(RvizRenderer& renderer, ftxui::Canvas& /*canvas*/, const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> /*tf_buffer*/) {
    if (!enabled_ || !model_loaded_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    auto root_link = model_.getRoot();
    if (root_link) render_link(root_link, renderer, fixed_frame);
}

void RobotModelDisplay::render_link(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, const std::string& fixed_frame) {
    if (!link) return;
    std::string frame_id = link->name, ns = node_->get_namespace();
    if (ns == "/") ns = "";
    try {
        geometry_msgs::msg::TransformStamped t_msg; bool found = false;
        try { t_msg = tf_buffer_->lookupTransform(fixed_frame, frame_id, tf2::TimePointZero); found = true; }
        catch (...) { if (frame_id[0] != '/' && !ns.empty()) {
            try { std::string ns_f = (ns[0] == '/' ? ns.substr(1) : ns) + "/" + frame_id;
                  t_msg = tf_buffer_->lookupTransform(fixed_frame, ns_f, tf2::TimePointZero); found = true; } catch (...) {}
        } }
        if (found) {
            tf2::Transform l_to_w; tf2::fromMsg(t_msg.transform, l_to_w);
            std::vector<urdf::VisualSharedPtr> visuals;
            if (!link->visual_array.empty()) visuals = link->visual_array; else if (link->visual) visuals.push_back(link->visual);
            for (const auto& visual : visuals) {
                if (!visual || !visual->geometry) continue;
                tf2::Transform v_to_l; const auto& o = visual->origin;
                v_to_l.setOrigin(tf2::Vector3(o.position.x, o.position.y, o.position.z));
                v_to_l.setRotation(tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));
                render_geometry(visual->geometry, l_to_w * v_to_l, renderer, visual->material);
            }
        }
    } catch (...) {}
    for (const auto& child : link->child_links) render_link(child, renderer, fixed_frame);
}

std::shared_ptr<Mesh> RobotModelDisplay::get_or_load_mesh(const std::string& uri) {
    if (mesh_cache_.count(uri)) return mesh_cache_[uri];
    std::string path = uri;
    if (uri.find("package://") == 0) {
        std::string sub = uri.substr(10); size_t pos = sub.find("/");
        if (pos != std::string::npos) {
            std::string pkg = sub.substr(0, pos), rel = sub.substr(pos);
            try { path = ament_index_cpp::get_package_share_directory(pkg) + rel; } catch (...) { return nullptr; }
        }
    } else if (uri.find("file://") == 0) path = uri.substr(7);
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_OptimizeMeshes | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->HasMeshes()) return nullptr;
    auto mesh_out = std::make_shared<Mesh>();
    process_assimp_node(scene, scene->mRootNode, aiMatrix4x4(), mesh_out);
    mesh_cache_[uri] = mesh_out;
    return mesh_out;
}

void RobotModelDisplay::process_assimp_node(const aiScene* scene, const aiNode* node, const aiMatrix4x4& transform, std::shared_ptr<Mesh> mesh_out) {
    aiMatrix4x4 cur_t = transform * node->mTransformation;
    std::default_random_engine gen(42); std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh* m_in = scene->mMeshes[node->mMeshes[i]];
        uint8_t mr = 255, mg = 255, mb = 255;
        if (scene->mNumMaterials > m_in->mMaterialIndex) {
            aiColor4D diff; if (AI_SUCCESS == aiGetMaterialColor(scene->mMaterials[m_in->mMaterialIndex], AI_MATKEY_COLOR_DIFFUSE, &diff)) {
                mr = static_cast<uint8_t>(diff.r * 255);
                mg = static_cast<uint8_t>(diff.g * 255);
                mb = static_cast<uint8_t>(diff.b * 255);
            }
        }
        for (unsigned int f = 0; f < m_in->mNumFaces; ++f) {
            const aiFace& face = m_in->mFaces[f];
            if (face.mNumIndices == 3) {
                aiVector3D v1 = cur_t * m_in->mVertices[face.mIndices[0]], v2 = cur_t * m_in->mVertices[face.mIndices[1]], v3 = cur_t * m_in->mVertices[face.mIndices[2]];
                tf2::Vector3 A(v1.x, v1.y, v1.z), B(v2.x, v2.y, v2.z), C(v3.x, v3.y, v3.z);
                float area = 0.5f * ((B-A).cross(C-A)).length();
                int num = std::clamp((int)(area * 160000.0f), 4, 100);
                for (int s=0; s<num; ++s) {
                    float r1 = dist(gen), r2 = dist(gen); if (r1+r2 > 1.0f) { r1=1-r1; r2=1-r2; }
                    float sr1 = std::sqrt(r1);
                    mesh_out->points.push_back({A + sr1*(1-r2)*(B-A) + sr1*r2*(C-A), mr, mg, mb});
                }
            }
        }
    }
    for (unsigned int i = 0; i < node->mNumChildren; ++i) process_assimp_node(scene, node->mChildren[i], cur_t, mesh_out);
}

void RobotModelDisplay::render_geometry(urdf::GeometrySharedPtr geom, const tf2::Transform& tf, RvizRenderer& renderer, urdf::MaterialSharedPtr mat) {
    uint8_t base_r = 255, base_g = 255, base_b = 255; 
    bool has_mat = false;
    float alpha = config_.alpha;
    if (mat) { 
        base_r = static_cast<uint8_t>(mat->color.r * 255);
        base_g = static_cast<uint8_t>(mat->color.g * 255);
        base_b = static_cast<uint8_t>(mat->color.b * 255);
        has_mat = true;
        alpha *= mat->color.a;
    }
    auto draw_pt_tf = [&](const tf2::Vector3& p_l, uint8_t r, uint8_t g, uint8_t b) { 
        tf2::Vector3 p = tf * p_l; 
        renderer.draw_point(p.x(), p.y(), p.z(), r, g, b, alpha); 
    };
    if (geom->type == urdf::Geometry::BOX) {
        auto box = std::static_pointer_cast<urdf::Box>(geom); float hx=box->dim.x/2, hy=box->dim.y/2, hz=box->dim.z/2;
        for(float x=-hx; x<=hx; x+=0.025f) for(float y=-hy; y<=hy; y+=0.025f) { 
            draw_pt_tf(tf2::Vector3(x,y,hz), base_r, base_g, base_b); 
            draw_pt_tf(tf2::Vector3(x,y,-hz), base_r, base_g, base_b); 
        }
    } else if (geom->type == urdf::Geometry::CYLINDER) {
        auto cyl = std::static_pointer_cast<urdf::Cylinder>(geom); float r=cyl->radius, hh=cyl->length/2;
        for (float z=-hh; z<=hh; z+=0.025f) for (int i=0; i<40; ++i) { 
            float a=(float)i/40*2*M_PI; 
            draw_pt_tf(tf2::Vector3(r*cos(a),r*sin(a),z), base_r, base_g, base_b); 
        }
    } else if (geom->type == urdf::Geometry::MESH) {
        auto m_u = std::static_pointer_cast<urdf::Mesh>(geom); auto mesh = get_or_load_mesh(m_u->filename); if (!mesh) return;
        tf2::Vector3 scale(m_u->scale.x, m_u->scale.y, m_u->scale.z);
        size_t max_r = 30000, step = std::max(1UL, mesh->points.size() / max_r);
        for (size_t i=0; i<mesh->points.size(); i+=step) {
            const auto& mp = mesh->points[i];
            tf2::Vector3 p_scaled(mp.pos.x()*scale.x(), mp.pos.y()*scale.y(), mp.pos.z()*scale.z());
            tf2::Vector3 p_w = tf * p_scaled;
            renderer.draw_point(p_w.x(), p_w.y(), p_w.z(), has_mat ? base_r : mp.r, has_mat ? base_g : mp.g, has_mat ? base_b : mp.b, alpha);
        }
    }
}

} // namespace terminal_rviz
