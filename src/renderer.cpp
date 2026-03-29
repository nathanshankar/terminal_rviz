#include "terminal_rviz/renderer.hpp"
#include <algorithm>

namespace terminal_rviz {

RvizRenderer::RvizRenderer() {}

void RvizRenderer::set_size(int width, int height) {
    if (width_ != width || height_ != height) {
        width_ = width;
        height_ = height;
        z_buffer_.assign(width_ * height_, 1000.0f);
        char_z_buffer_.assign((width_ / 2) * (height_ / 4), 1000.0f);
        char_colors_.assign((width_ / 2) * (height_ / 4), ftxui::Color::White);
        dirty_cells_.clear();
    }
}

void RvizRenderer::set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz) {
    yaw_ = yaw; pitch_ = pitch; roll_ = roll; dist_ = dist; cx_ = cx; cy_ = cy; cz_ = cz;
}

void RvizRenderer::clear() {
    // Only clear cells that were actually used
    for (int idx : dirty_cells_) {
        char_z_buffer_[idx] = 1000.0f;
        // Also need to clear the specific 8 dots in z_buffer_ for this cell
        int cw = width_ / 2;
        int cx = idx % cw, cy = idx / cw;
        for (int dy = 0; dy < 4; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                z_buffer_[(cy * 4 + dy) * width_ + (cx * 2 + dx)] = 1000.0f;
            }
        }
    }
    dirty_cells_.clear();
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
        float z_inv = zoom_ / out_z;
        out_sx = (width_ / 2) + static_cast<int>(x3 * z_inv);
        out_sy = (height_ / 2) + static_cast<int>(y3 * z_inv);
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
        int cw = width_ / 2;
        int c_idx = cy * cw + cx;
        if (z < char_z_buffer_[c_idx]) {
            if (char_z_buffer_[c_idx] >= 1000.0f) dirty_cells_.push_back(c_idx);
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

    // Fast 2D Path for lines
    if (p1 && p2) {
        int dx = std::abs(sx2 - sx1), dy = std::abs(sy2 - sy1);
        int sx = (sx1 < sx2) ? 1 : -1, sy = (sy1 < sy2) ? 1 : -1;
        int err = dx - dy;
        int x = sx1, y = sy1;
        while (true) {
            float t = (dx > dy) ? (float)std::abs(x - sx1) / dx : (float)std::abs(y - sy1) / dy;
            plot(x, y, sz1 + (sz2 - sz1) * t, color);
            if (x == sx2 && y == sy2) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    } else {
        // Fallback for partially clipped lines
        float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
        for (int i = 0; i <= 50; ++i) {
            float t = (float)i / 50.0f;
            draw_point(x1 + dx * t, y1 + dy * t, z1 + dz * t, color);
        }
    }
}

void RvizRenderer::finish(ftxui::Canvas& canvas) {
    int cw = width_ / 2;
    for (int c_idx : dirty_cells_) {
        ftxui::Color col = char_colors_[c_idx];
        int cx = c_idx % cw, cy = c_idx / cw;
        for (int dy = 0; dy < 4; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                int px = cx * 2 + dx, py = cy * 4 + dy;
                if (z_buffer_[py * width_ + px] < 1000.0f) canvas.DrawPoint(px, py, true, col);
            }
        }
    }
}

bool RvizRenderer::pick_ground_plane(int sx, int sy, float& out_x, float& out_y) const {
    if (width_ == 0 || height_ == 0) return false;
    float nx = (static_cast<float>(sx) - (width_ / 2.0f)) / zoom_;
    float ny = (static_cast<float>(sy) - (height_ / 2.0f)) / zoom_;
    float s_yaw = std::sin(yaw_), c_yaw = std::cos(yaw_);
    float s_pitch = std::sin(pitch_), c_pitch = std::cos(pitch_);
    float s_roll = std::sin(roll_), c_roll = std::cos(roll_);
    auto unrotate = [&](float x, float y, float z, float& ox, float& oy, float& oz) {
        float x1 = x * c_roll + y * s_roll, y1 = -x * s_roll + y * c_roll, z1 = z;
        float x2 = x1, y2 = y1 * c_pitch + z1 * s_pitch, z2 = -y1 * s_pitch + z1 * c_pitch;
        float rx = x2 * c_yaw - z2 * s_yaw, ry = y2, rz = x2 * s_yaw + z2 * c_yaw;
        ox = rz + cx_; oy = -rx + cy_; oz = -ry + cz_;
    };
    float cxw, cyw, czw; unrotate(0, 0, -dist_, cxw, cyw, czw);
    float rxw, ryw, rzw; unrotate(nx, ny, 1.0f - dist_, rxw, ryw, rzw);
    float dx = rxw - cxw, dy = ryw - cyw, dz = rzw - czw;
    if (std::abs(dz) < 1e-6) return false;
    float t = -czw / dz; if (t < 0) return false;
    out_x = cxw + t * dx; out_y = cyw + t * dy;
    return true;
}

} // namespace terminal_rviz
