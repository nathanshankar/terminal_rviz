#include "terminal_rviz/displays/robot_model_display.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>

namespace terminal_rviz {

RobotModelDisplay::RobotModelDisplay(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : Display("RobotModel", node), tf_buffer_(tf_buffer) {}

void RobotModelDisplay::onInitialize() {
    topic_ = "robot_description";
    setTopic(topic_);
}

void RobotModelDisplay::setTopic(const std::string& topic) {
    topic_ = topic;
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic_, rclcpp::QoS(1).transient_local(), std::bind(&RobotModelDisplay::callback, this, std::placeholders::_1));
}

void RobotModelDisplay::callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (model_.initString(msg->data)) {
        model_loaded_ = true;
        mesh_cache_.clear();
    }
}

void RobotModelDisplay::render(RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!enabled_ || !model_loaded_) return;

    std::lock_guard<std::mutex> lock(mtx_);
    auto root_link = model_.getRoot();
    if (root_link) {
        render_link(root_link, renderer, canvas, fixed_frame);
    }
}

void RobotModelDisplay::render_link(urdf::LinkConstSharedPtr link, RvizRenderer& renderer, ftxui::Canvas& canvas, const std::string& fixed_frame) {
    if (!link) return;

    std::string frame_id = link->name;
    std::string ns = node_->get_namespace();
    if (ns == "/") ns = "";

    try {
        geometry_msgs::msg::TransformStamped transform_msg;
        bool found = false;

        try {
            transform_msg = tf_buffer_->lookupTransform(fixed_frame, frame_id, tf2::TimePointZero);
            found = true;
        } catch (...) {
            if (frame_id[0] != '/' && !ns.empty()) {
                try {
                    std::string ns_frame = (ns[0] == '/' ? ns.substr(1) : ns) + "/" + frame_id;
                    transform_msg = tf_buffer_->lookupTransform(fixed_frame, ns_frame, tf2::TimePointZero);
                    found = true;
                } catch (...) {}
            }
        }

        if (found) {
            tf2::Transform link_to_world;
            tf2::fromMsg(transform_msg.transform, link_to_world);

            std::vector<urdf::VisualSharedPtr> visuals;
            if (!link->visual_array.empty()) visuals = link->visual_array;
            else if (link->visual) visuals.push_back(link->visual);

            for (const auto& visual : visuals) {
                if (!visual || !visual->geometry) continue;
                tf2::Transform visual_to_link;
                const auto& o = visual->origin;
                visual_to_link.setOrigin(tf2::Vector3(o.position.x, o.position.y, o.position.z));
                visual_to_link.setRotation(tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));
                render_geometry(visual->geometry, link_to_world * visual_to_link, renderer, canvas, visual->material);
            }
        }
    } catch (...) {}

    for (const auto& child : link->child_links) {
        render_link(child, renderer, canvas, fixed_frame);
    }
}

std::shared_ptr<Mesh> RobotModelDisplay::get_or_load_mesh(const std::string& uri) {
    if (mesh_cache_.count(uri)) return mesh_cache_[uri];

    std::string path = uri;
    if (uri.find("package://") == 0) {
        std::string sub = uri.substr(10);
        size_t pos = sub.find("/");
        if (pos != std::string::npos) {
            std::string pkg = sub.substr(0, pos);
            std::string rel = sub.substr(pos);
            try {
                path = ament_index_cpp::get_package_share_directory(pkg) + rel;
            } catch (...) { return nullptr; }
        }
    } else if (uri.find("file://") == 0) {
        path = uri.substr(7);
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_OptimizeMeshes | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->HasMeshes()) return nullptr;

    auto mesh_out = std::make_shared<Mesh>();
    process_assimp_node(scene, scene->mRootNode, aiMatrix4x4(), mesh_out);

    mesh_cache_[uri] = mesh_out;
    return mesh_out;
}

void RobotModelDisplay::process_assimp_node(const aiScene* scene, const aiNode* node, const aiMatrix4x4& transform, std::shared_ptr<Mesh> mesh_out) {
    aiMatrix4x4 current_transform = transform * node->mTransformation;
    
    std::default_random_engine gen(42);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    const float density = 80000.0f; 

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh* mesh_in = scene->mMeshes[node->mMeshes[i]];
        
        // Get material color if available
        ftxui::Color mesh_color = ftxui::Color::White;
        if (scene->mNumMaterials > mesh_in->mMaterialIndex) {
            aiColor4D diffuse;
            if (AI_SUCCESS == aiGetMaterialColor(scene->mMaterials[mesh_in->mMaterialIndex], AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
                mesh_color = ftxui::Color::RGB(int(diffuse.r * 255), int(diffuse.g * 255), int(diffuse.b * 255));
            }
        }

        for (unsigned int f = 0; f < mesh_in->mNumFaces; ++f) {
            const aiFace& face = mesh_in->mFaces[f];
            if (face.mNumIndices == 3) {
                aiVector3D v1 = current_transform * mesh_in->mVertices[face.mIndices[0]];
                aiVector3D v2 = current_transform * mesh_in->mVertices[face.mIndices[1]];
                aiVector3D v3 = current_transform * mesh_in->mVertices[face.mIndices[2]];
                
                tf2::Vector3 A(v1.x, v1.y, v1.z);
                tf2::Vector3 B(v2.x, v2.y, v2.z);
                tf2::Vector3 C(v3.x, v3.y, v3.z);

                float area = 0.5f * ((B - A).cross(C - A)).length();
                if (area < 1e-7) continue;

                int num_samples = std::clamp((int)(area * density), 2, 50); 

                for (int s = 0; s < num_samples; ++s) {
                    float r1 = dist(gen), r2 = dist(gen);
                    if (r1 + r2 > 1.0f) { r1 = 1.0f - r1; r2 = 1.0f - r2; }
                    
                    float sqrt_r1 = std::sqrt(r1);
                    float u = 1.0f - sqrt_r1;
                    float v = sqrt_r1 * (1.0f - r2);
                    float w = sqrt_r1 * r2;
                    
                    tf2::Vector3 P = u*A + v*B + w*C;
                    mesh_out->points.push_back({P, mesh_color});
                }
            }
        }
    }

    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        process_assimp_node(scene, node->mChildren[i], current_transform, mesh_out);
    }
}

void RobotModelDisplay::render_geometry(urdf::GeometrySharedPtr geom, const tf2::Transform& tf, RvizRenderer& renderer, ftxui::Canvas& canvas, urdf::MaterialSharedPtr material) {
    ftxui::Color base_color = ftxui::Color::White;
    bool has_urdf_color = false;
    if (material) {
        base_color = ftxui::Color::RGB(int(material->color.r * 255), int(material->color.g * 255), int(material->color.b * 255));
        has_urdf_color = true;
    }

    auto draw_point_tf = [&](const tf2::Vector3& p_local, ftxui::Color col) {
        tf2::Vector3 p = tf * p_local;
        renderer.draw_point(p.x(), p.y(), p.z(), col, canvas);
    };

    if (geom->type == urdf::Geometry::BOX) {
        auto box = std::static_pointer_cast<urdf::Box>(geom);
        float hx = box->dim.x/2, hy = box->dim.y/2, hz = box->dim.z/2;
        float step = 0.05f;
        for(float x=-hx; x<=hx; x+=step) for(float y=-hy; y<=hy; y+=step) {
            draw_point_tf(tf2::Vector3(x, y,  hz), base_color);
            draw_point_tf(tf2::Vector3(x, y, -hz), base_color);
        }
    } 
    else if (geom->type == urdf::Geometry::CYLINDER) {
        auto cyl = std::static_pointer_cast<urdf::Cylinder>(geom);
        float r = cyl->radius, hh = cyl->length/2;
        for (float z=-hh; z<=hh; z+=0.05f) {
            for (int i=0; i<20; ++i) {
                float a = (float)i/20*2*M_PI;
                draw_point_tf(tf2::Vector3(r*cos(a), r*sin(a), z), base_color);
            }
        }
    }
    else if (geom->type == urdf::Geometry::MESH) {
        auto m_urdf = std::static_pointer_cast<urdf::Mesh>(geom);
        auto mesh = get_or_load_mesh(m_urdf->filename);
        if (!mesh) return;
        
        tf2::Vector3 scale(m_urdf->scale.x, m_urdf->scale.y, m_urdf->scale.z);
        size_t max_render = 15000; 
        size_t step = std::max(1UL, mesh->points.size() / max_render);
        
        for (size_t i = 0; i < mesh->points.size(); i += step) {
            const auto& mp = mesh->points[i];
            tf2::Vector3 p_scaled(mp.pos.x()*scale.x(), mp.pos.y()*scale.y(), mp.pos.z()*scale.z());
            tf2::Vector3 p_world = tf * p_scaled;
            
            ftxui::Color final_col = has_urdf_color ? base_color : mp.color;
            
            // Simple depth shading multiplier
            float intensity = std::clamp(static_cast<float>(1.0f - (p_world.z() * 0.1f)), 0.4f, 1.0f);
            // Since ftxui::Color doesn't have a simple multiplier, we just use the color as is 
            // or we could manually scale RGB. Let's keep the true color for now as requested.
            
            renderer.draw_point(p_world.x(), p_world.y(), p_world.z(), final_col, canvas);
        }
    }
}

} // namespace terminal_rviz
