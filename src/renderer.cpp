#include "terminal_rviz/renderer.hpp"

namespace terminal_rviz {

RvizRenderer::RvizRenderer() {}

void RvizRenderer::set_size(int width, int height) {
    if (width_ != width || height_ != height) {
        width_ = width;
        height_ = height;
        z_buffer_.assign(width_ * height_, 1000.0f);
        char_z_buffer_.assign((width_ / 2) * (height_ / 4), 1000.0f);
        char_colors_.assign((width_ / 2) * (height_ / 4), ftxui::Color::White);
    }
}

void RvizRenderer::set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz) {
    yaw_ = yaw; pitch_ = pitch; roll_ = roll; dist_ = dist; cx_ = cx; cy_ = cy; cz_ = cz;
}

void RvizRenderer::clear() {
    std::fill(z_buffer_.begin(), z_buffer_.end(), 1000.0f);
    std::fill(char_z_buffer_.begin(), char_z_buffer_.end(), 1000.0f);
}

bool RvizRenderer::project(float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) const {
    float tx = dx - cx_, ty = dy - cy_, tz = dz - cz_;
    float rx = -ty, ry = -tz, rz = tx;
    float s_yaw = std::sin(yaw_), c_yaw = std::cos(yaw_);
    float s_pitch = std::sin(pitch_), c_pitch = std::cos(pitch_);
    float s_roll = std::sin(roll_), c_roll = std::cos(roll_);
    float x1 = rx * c_yaw + rz * s_yaw, z1 = -rx * s_yaw + rz * c_yaw;
    float y2 = ry * c_pitch - z1 * s_pitch, z2 = ry * s_pitch + z1 * c_pitch;
    float x3 = x1 * c_roll - y2 * s_roll, y3 = x1 * s_roll + y2 * c_roll;
    out_z = z2 + dist_;
    if (out_z > 0.1f) {
        float z_inv = 1.0f / out_z;
        out_sx = (width_ / 2) + static_cast<int>(zoom_ * x3 * z_inv);
        out_sy = (height_ / 2) + static_cast<int>(zoom_ * y3 * z_inv);
        return true;
    }
    return false;
}

void RvizRenderer::plot(int x, int y, float z, ftxui::Color color) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    int idx = y * width_ + x;
    if (z < z_buffer_[idx]) {
        z_buffer_[idx] = z;
        int cx = x / 2, cy = y / 4;
        int c_idx = cy * (width_ / 2) + cx;
        if (z < char_z_buffer_[c_idx]) {
            char_z_buffer_[c_idx] = z;
            char_colors_[c_idx] = color;
        }
    }
}

void RvizRenderer::draw_point(float x, float y, float z, ftxui::Color color) {
    int sx, sy; float sz;
    if (project(x, y, z, sx, sy, sz)) plot(sx, sy, sz, color);
}

void RvizRenderer::draw_line(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color color) {
    int sx1, sy1, sx2, sy2; float sz1, sz2;
    bool p1 = project(x1, y1, z1, sx1, sy1, sz1);
    bool p2 = project(x2, y2, z2, sx2, sy2, sz2);
    if (!p1 && !p2) return;
    int steps = (p1 && p2) ? std::max(std::abs(sx2 - sx1), std::abs(sy2 - sy1)) : 100;
    steps = std::clamp(steps, 2, 1000);
    float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        draw_point(x1 + dx * t, y1 + dy * t, z1 + dz * t, color);
    }
}

void RvizRenderer::finish(ftxui::Canvas& canvas) {
    int cw = width_ / 2, ch = height_ / 4;
    for (int cy = 0; cy < ch; ++cy) {
        for (int cx = 0; cx < cw; ++cx) {
            int c_idx = cy * cw + cx;
            if (char_z_buffer_[c_idx] < 1000.0f) {
                ftxui::Color c = char_colors_[c_idx];
                for (int dy = 0; dy < 4; ++dy) {
                    for (int dx = 0; dx < 2; ++dx) {
                        int px = cx * 2 + dx, py = cy * 4 + dy;
                        if (z_buffer_[py * width_ + px] < 1000.0f) canvas.DrawPoint(px, py, true, c);
                    }
                }
            }
        }
    }
}

} // namespace terminal_rviz
