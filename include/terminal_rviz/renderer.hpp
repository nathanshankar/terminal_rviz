#ifndef TERMINAL_RVIZ_RENDERER_HPP_
#define TERMINAL_RVIZ_RENDERER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include "ftxui/screen/color.hpp"
#include "ftxui/dom/canvas.hpp"
#include <tf2/LinearMath/Transform.h>

namespace terminal_rviz {

class RvizRenderer {
public:
    RvizRenderer();
    ~RvizRenderer();

    void set_size(int width, int height);
    void set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz);
    void set_zoom(float zoom) { zoom_ = zoom; }
    
    void clear();
    void set_global_alpha(float alpha) { global_alpha_ = alpha; }
    
    void draw_point(float x, float y, float z, ftxui::Color color, float alpha = 1.0f);
    void draw_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void draw_line(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color color, float alpha = 1.0f);
    void draw_line(float x1, float y1, float z1, float x2, float y2, float z2, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void draw_box(float x, float y, float z, float sx, float sy, float sz, ftxui::Color color, float alpha = 1.0f);
    void draw_box(float x, float y, float z, float sx, float sy, float sz, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void draw_sphere(float x, float y, float z, float radius, ftxui::Color color, float alpha = 1.0f);
    void draw_sphere(float x, float y, float z, float radius, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void draw_tile(float x, float y, float z, float sx, float sy, ftxui::Color color, float alpha = 1.0f);
    void draw_tile(float x, float y, float z, float sx, float sy, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void draw_arrow(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color color, float alpha = 1.0f, float head_scale = 1.0f);
    void draw_arrow(float x1, float y1, float z1, float x2, float y2, float z2, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f, float head_scale = 1.0f);

    void draw_circle(float x, float y, float z, float radius, ftxui::Color color, float alpha = 1.0f);
    void draw_circle(float x, float y, float z, float radius, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);

    void plot(int x, int y, float z, ftxui::Color color, float alpha = 1.0f);
    void plot(int x, int y, float z, uint8_t r, uint8_t g, uint8_t b, float alpha = 1.0f);
    void finish(ftxui::Canvas& canvas);

    struct Dot {
        float z = 1000.0f;
        uint8_t r = 255, g = 255, b = 255;
        uint8_t padding = 0; // Explicit padding for 4-byte alignment of next member
        float alpha = 1.0f;
    };

    struct Label {
        float x, y, z;
        std::string text;
        ftxui::Color color;
    };
    void draw_text(float x, float y, float z, const std::string& text, ftxui::Color color) {
        labels_.push_back({x, y, z, text, color});
    }
    const std::vector<Label>& get_labels() const { return labels_; }

    bool project(float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) const;
    
    struct Projector {
        float m[3][4];
        float zoom;
        int width, height;
        inline bool project(float x, float y, float z, int& sx, int& sy, float& sz) const {
            sz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3];
            if (sz > 0.1f) {
                float px = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3];
                float py = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3];
                float z_inv = zoom / sz;
                sx = (width / 2) + static_cast<int>(px * z_inv);
                sy = (height / 2) + static_cast<int>(py * z_inv);
                return true;
            }
            return false;
        }
    };
    Projector get_projector(const tf2::Transform& world_to_object) const;
    Projector get_view_projector() const;

    void enable_gpu(bool enable);
    bool is_gpu_enabled() const { return use_gpu_; }
    void gpu_render_points(const std::vector<float>& points, const std::vector<uint8_t>& colors, const Projector& projector, float alpha);
    void gpu_render_lines(const std::vector<float>& lines, const std::vector<uint8_t>& colors, const Projector& projector, float alpha);

    bool pick_ground_plane(int sx, int sy, float& out_x, float& out_y) const;

    int get_width() const { return width_; }
    int get_height() const { return height_; }

private:
    int width_ = 0;
    int height_ = 0;
    
    float yaw_ = 0.0f, pitch_ = 0.0f, roll_ = 0.0f, dist_ = 5.0f, zoom_ = 100.0f;
    float cx_ = 0.0f, cy_ = 0.0f, cz_ = 0.0f;
    float global_alpha_ = 1.0f;

    // Pre-calculated rotation matrix
    float m00_, m01_, m02_;
    float m10_, m11_, m12_;
    float m20_, m21_, m22_;

    std::vector<Label> labels_;
    std::vector<Dot> dot_buffer_;
    std::vector<float> char_z_buffer_;
    std::vector<ftxui::Color> char_colors_;
    std::vector<int> dirty_cells_;
    std::vector<bool> is_dirty_;

    std::vector<float> gpu_points_3d_;
    std::vector<uint8_t> gpu_points_color_;
    std::vector<float> gpu_lines_3d_;
    std::vector<uint8_t> gpu_lines_color_;

    bool use_gpu_ = false;
#ifdef USE_GPU
    std::unique_ptr<class GpuRvizRenderer> gpu_renderer_;
#endif
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_RENDERER_HPP_
