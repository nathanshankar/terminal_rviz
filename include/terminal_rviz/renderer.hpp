#ifndef TERMINAL_RVIZ_RENDERER_HPP_
#define TERMINAL_RVIZ_RENDERER_HPP_

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <mutex>

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
    
    void draw_point(float x, float y, float z, ftxui::Color color, ftxui::Canvas& canvas);
    void draw_line(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color color, ftxui::Canvas& canvas);
    
    bool project(float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) const;

private:
    void plot_z_point(int x, int y, float z, ftxui::Color color, ftxui::Canvas& canvas);

    int width_ = 0;
    int height_ = 0;
    
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
    float roll_ = 0.0f;
    float dist_ = 5.0f;
    float zoom_ = 100.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;
    float cz_ = 0.0f;

    std::vector<float> z_buffer_;
};

} // namespace terminal_rviz

#endif // TERMINAL_RVIZ_RENDERER_HPP_
