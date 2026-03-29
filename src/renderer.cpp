#include "terminal_rviz/renderer.hpp"

namespace terminal_rviz {

RvizRenderer::RvizRenderer() {}

void RvizRenderer::set_size(int width, int height) {
    if (width_ != width || height_ != height) {
        width_ = width;
        height_ = height;
        z_buffer_.assign(width_ * height_, 1000.0f);
    }
}

void RvizRenderer::set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz) {
    yaw_ = yaw;
    pitch_ = pitch;
    roll_ = roll;
    dist_ = dist;
    cx_ = cx;
    cy_ = cy;
    cz_ = cz;
}

void RvizRenderer::clear() {
    std::fill(z_buffer_.begin(), z_buffer_.end(), 1000.0f);
}

bool RvizRenderer::project(float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) const {
    float tx = dx - cx_;
    float ty = dy - cy_;
    float tz = dz - cz_;

    // Standard ROS coords: X forward, Y left, Z up
    // Convert to camera coords (Z forward, X right, Y down) for projection
    float rx = -ty;
    float ry = -tz;
    float rz = tx;

    const float s_yaw = std::sin(yaw_);
    const float c_yaw = std::cos(yaw_);
    const float s_pitch = std::sin(pitch_);
    const float c_pitch = std::cos(pitch_);
    const float s_roll = std::sin(roll_);
    const float c_roll = std::cos(roll_);

    float x1 = rx * c_yaw + rz * s_yaw;
    float z1 = -rx * s_yaw + rz * c_yaw;
    float y2 = ry * c_pitch - z1 * s_pitch;
    float z2 = ry * s_pitch + z1 * c_pitch;
    float x3 = x1 * c_roll - y2 * s_roll;
    float y3 = x1 * s_roll + y2 * c_roll;

    out_z = z2 + dist_;
    if (out_z > 0.1f) {
        float z_inv = 1.0f / out_z;
        out_sx = (width_ / 2) + static_cast<int>(zoom_ * x3 * z_inv);
        out_sy = (height_ / 2) + static_cast<int>(zoom_ * y3 * z_inv);
        return true;
    }
    return false;
}

void RvizRenderer::plot_z_point(int x, int y, float z, ftxui::Color color, ftxui::Canvas& canvas) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    int idx = y * width_ + x;
    if (z < z_buffer_[idx]) {
        z_buffer_[idx] = z;
        canvas.DrawPoint(x, y, true, color);
    }
}

void RvizRenderer::draw_point(float x, float y, float z, ftxui::Color color, ftxui::Canvas& canvas) {
    int sx, sy;
    float sz;
    if (project(x, y, z, sx, sy, sz)) {
        plot_z_point(sx, sy, sz, color, canvas);
    }
}

void RvizRenderer::draw_line(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color color, ftxui::Canvas& canvas) {
    int sx1, sy1, sx2, sy2;
    float sz1, sz2;
    
    bool p1 = project(x1, y1, z1, sx1, sy1, sz1);
    bool p2 = project(x2, y2, z2, sx2, sy2, sz2);

    if (!p1 && !p2) return;

    // Simple line stepping in 3D
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    
    // Determine number of steps based on screen distance
    int steps = 0;
    if (p1 && p2) {
        steps = std::max(std::abs(sx2 - sx1), std::abs(sy2 - sy1));
    } else {
        steps = 100; // Arbitrary for partially visible lines
    }
    steps = std::clamp(steps, 2, 1000);

    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        draw_point(x1 + dx * t, y1 + dy * t, z1 + dz * t, color, canvas);
    }
}

} // namespace terminal_rviz
