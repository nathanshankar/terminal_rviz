#ifndef TERMINAL_RVIZ_RENDERER_HPP_
#define TERMINAL_RVIZ_RENDERER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include "ftxui/screen/color.hpp"
#include "ftxui/dom/canvas.hpp"

namespace terminal_rviz {

class RvizRenderer {
public:
    RvizRenderer();

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
    bool pick_ground_plane(int sx, int sy, float& out_x, float& out_y) const;

    int get_width() const { return width_; }
    int get_height() const { return height_; }

private:
    int width_ = 0;
    int height_ = 0;
    
    float yaw_ = 0.0f, pitch_ = 0.0f, roll_ = 0.0f, dist_ = 5.0f, zoom_ = 100.0f;
    float cx_ = 0.0f, cy_ = 0.0f, cz_ = 0.0f;
    float global_alpha_ = 1.0f;

    std::vector<Label> labels_;
    std::vector<float> z_buffer_;
    std::vector<uint8_t> r_buffer_;
    std::vector<uint8_t> g_buffer_;
    std::vector<uint8_t> b_buffer_;
    std::vector<float> dot_alphas_;
    std::vector<float> char_z_buffer_;
    std::vector<ftxui::Color> char_colors_;
    std::vector<int> dirty_cells_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_RENDERER_HPP_
