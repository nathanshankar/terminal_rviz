#include "terminal_rviz/robot_renderer.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace terminal_rviz {

RobotRenderer::RobotRenderer() {}

bool RobotRenderer::init_urdf(const std::string& urdf_xml) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (model_.initString(urdf_xml)) {
        model_loaded_ = true;
        mesh_cache_.clear();
        return true;
    }
    return false;
}

void RobotRenderer::render(RvizRenderer& renderer, const std::string& fixed_frame, 
                          std::shared_ptr<tf2_ros::Buffer> tf_buffer, float alpha) {
    if (!model_loaded_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    auto root_link = model_.getRoot();
    if (root_link) render_link(root_link, renderer, fixed_frame, tf_buffer, alpha);
}

void RobotRenderer::render_link(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, 
                               const std::string& fixed_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer, float alpha) {
    if (!link) return;
    try {
        auto t_msg = tf_buffer->lookupTransform(fixed_frame, link->name, tf2::TimePointZero);
        tf2::Transform l_to_w; tf2::fromMsg(t_msg.transform, l_to_w);
        
        std::vector<urdf::VisualSharedPtr> visuals;
        if (!link->visual_array.empty()) visuals = link->visual_array; 
        else if (link->visual) visuals.push_back(link->visual);
        
        for (const auto& visual : visuals) {
            if (!visual || !visual->geometry) continue;
            tf2::Transform v_to_l; const auto& o = visual->origin;
            v_to_l.setOrigin(tf2::Vector3(o.position.x, o.position.y, o.position.z));
            v_to_l.setRotation(tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));
            render_geometry(visual->geometry, l_to_w * v_to_l, renderer, visual->material, alpha);
        }
    } catch (...) {}
    for (const auto& child : link->child_links) render_link(child, renderer, fixed_frame, tf_buffer, alpha);
}

void RobotRenderer::render_state(RvizRenderer& renderer, const std::string& fixed_frame, 
                                std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                const std::map<std::string, double>& joint_positions, float alpha) {
    if (!model_loaded_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    auto root_link = model_.getRoot();
    if (!root_link) return;

    tf2::Transform root_to_world = tf2::Transform::getIdentity();
    try {
        auto t_msg = tf_buffer->lookupTransform(fixed_frame, root_link->name, tf2::TimePointZero);
        tf2::fromMsg(t_msg.transform, root_to_world);
    } catch (...) {}

    render_link_recursive(root_link, renderer, root_to_world, joint_positions, alpha);
}

void RobotRenderer::render_link_recursive(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, 
                                         const tf2::Transform& parent_tf, 
                                         const std::map<std::string, double>& joint_positions, float alpha) {
    if (!link) return;
    
    std::vector<urdf::VisualSharedPtr> visuals;
    if (!link->visual_array.empty()) visuals = link->visual_array; 
    else if (link->visual) visuals.push_back(link->visual);
    
    for (const auto& visual : visuals) {
        if (!visual || !visual->geometry) continue;
        tf2::Transform v_to_l; const auto& o = visual->origin;
        v_to_l.setOrigin(tf2::Vector3(o.position.x, o.position.y, o.position.z));
        v_to_l.setRotation(tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));
        render_geometry(visual->geometry, parent_tf * v_to_l, renderer, visual->material, alpha);
    }

    for (const auto& child_joint : link->child_joints) {
        if (!child_joint) continue;
        tf2::Transform j_tf;
        const auto& o = child_joint->parent_to_joint_origin_transform;
        j_tf.setOrigin(tf2::Vector3(o.position.x, o.position.y, o.position.z));
        j_tf.setRotation(tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));
        
        if (child_joint->type != urdf::Joint::FIXED) {
            double pos = 0.0;
            if (joint_positions.count(child_joint->name)) pos = joint_positions.at(child_joint->name);
            
            tf2::Quaternion q;
            tf2::Vector3 axis(child_joint->axis.x, child_joint->axis.y, child_joint->axis.z);
            if (child_joint->type == urdf::Joint::REVOLUTE || child_joint->type == urdf::Joint::CONTINUOUS) {
                q.setRotation(axis, pos);
            } else if (child_joint->type == urdf::Joint::PRISMATIC) {
                j_tf.setOrigin(j_tf.getOrigin() + axis * pos);
            }
            j_tf.setRotation(j_tf.getRotation() * q);
        }
        
        auto child_link = model_.getLink(child_joint->child_link_name);
        if (child_link) render_link_recursive(child_link, renderer, parent_tf * j_tf, joint_positions, alpha);
    }
}

void RobotRenderer::render_geometry(urdf::GeometrySharedPtr geom, const tf2::Transform& tf, 
                                   RvizRenderer& renderer, urdf::MaterialSharedPtr mat, float alpha) {
    uint8_t base_r = 255, base_g = 255, base_b = 255; 
    bool has_mat = false;
    if (mat) { 
        base_r = static_cast<uint8_t>(mat->color.r * 255);
        base_g = static_cast<uint8_t>(mat->color.g * 255);
        base_b = static_cast<uint8_t>(mat->color.b * 255);
        has_mat = true;
        alpha *= mat->color.a;
    }

    if (geom->type == urdf::Geometry::BOX) {
        auto box = std::static_pointer_cast<urdf::Box>(geom); 
        float hx = box->dim.x / 2.0f, hy = box->dim.y / 2.0f, hz = box->dim.z / 2.0f;
        for (float x = -hx; x <= hx; x += 0.05f) {
            for (float y = -hy; y <= hy; y += 0.05f) {
                tf2::Vector3 p1 = tf * tf2::Vector3(x, y, hz);
                renderer.draw_point(p1.x(), p1.y(), p1.z(), base_r, base_g, base_b, alpha);
                tf2::Vector3 p2 = tf * tf2::Vector3(x, y, -hz);
                renderer.draw_point(p2.x(), p2.y(), p2.z(), base_r, base_g, base_b, alpha);
            }
        }
    } else if (geom->type == urdf::Geometry::MESH) {
        auto m_u = std::static_pointer_cast<urdf::Mesh>(geom); 
        auto mesh = get_or_load_mesh(m_u->filename); if (!mesh) return;
        tf2::Vector3 scale(m_u->scale.x, m_u->scale.y, m_u->scale.z);
        size_t max_r = 15000, step = std::max(1UL, mesh->points.size() / max_r);
        for (size_t i=0; i<mesh->points.size(); i+=step) {
            const auto& mp = mesh->points[i];
            tf2::Vector3 p_w = tf * tf2::Vector3(mp.pos.x()*scale.x(), mp.pos.y()*scale.y(), mp.pos.z()*scale.z());
            renderer.draw_point(p_w.x(), p_w.y(), p_w.z(), has_mat ? base_r : mp.r, has_mat ? base_g : mp.g, has_mat ? base_b : mp.b, alpha);
        }
    }
}

std::shared_ptr<Mesh> RobotRenderer::get_or_load_mesh(const std::string& uri) {
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
    aiMatrix4x4 root_transform;
    if (uri.find(".dae") != std::string::npos) aiMatrix4x4::RotationX(M_PI / 2.f, root_transform);
    process_assimp_node(scene, scene->mRootNode, root_transform, mesh_out);
    mesh_cache_[uri] = mesh_out;
    return mesh_out;
}

void RobotRenderer::process_assimp_node(const aiScene* scene, const aiNode* node, const aiMatrix4x4& transform, std::shared_ptr<Mesh> mesh_out) {
    aiMatrix4x4 cur_t = transform * node->mTransformation;
    std::default_random_engine gen(42); std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh* m_in = scene->mMeshes[node->mMeshes[i]];
        uint8_t mr = 255, mg = 255, mb = 255;
        if (scene->mNumMaterials > m_in->mMaterialIndex) {
            aiColor4D diff; if (AI_SUCCESS == aiGetMaterialColor(scene->mMaterials[m_in->mMaterialIndex], AI_MATKEY_COLOR_DIFFUSE, &diff)) {
                mr = static_cast<uint8_t>(diff.r * 255); mg = static_cast<uint8_t>(diff.g * 255); mb = static_cast<uint8_t>(diff.b * 255);
            }
        }
        for (unsigned int f = 0; f < m_in->mNumFaces; ++f) {
            const aiFace& face = m_in->mFaces[f];
            if (face.mNumIndices == 3) {
                aiVector3D v1 = cur_t * m_in->mVertices[face.mIndices[0]], v2 = cur_t * m_in->mVertices[face.mIndices[1]], v3 = cur_t * m_in->mVertices[face.mIndices[2]];
                tf2::Vector3 A(v1.x, v1.y, v1.z), B(v2.x, v2.y, v2.z), C(v3.x, v3.y, v3.z);
                float area = 0.5f * ((B-A).cross(C-A)).length();
                int num = std::clamp((int)(area * 100000.0f), 5, 100);
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

} // namespace terminal_rviz
