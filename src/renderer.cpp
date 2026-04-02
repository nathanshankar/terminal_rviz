#include "terminal_rviz/renderer.hpp"
#include <algorithm>

namespace terminal_rviz {

RvizRenderer::RvizRenderer() {}

void RvizRenderer::set_size(int width, int height) {
    if (width_ != width || height_ != height) {
        width_ = width;
        height_ = height;
        z_buffer_.assign(width_ * height_, 1000.0f);
        r_buffer_.assign(width_ * height_, 255);
        g_buffer_.assign(width_ * height_, 255);
        b_buffer_.assign(width_ * height_, 255);
        dot_alphas_.assign(width_ * height_, 1.0f);
        char_z_buffer_.assign((width_ / 2) * (height_ / 4), 1000.0f);
        char_colors_.assign((width_ / 2) * (height_ / 4), ftxui::Color::White);
        dirty_cells_.clear();
    }
}

void RvizRenderer::set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz) {
    yaw_ = yaw; pitch_ = pitch; roll_ = roll; dist_ = dist; cx_ = cx; cy_ = cy; cz_ = cz;
}

void RvizRenderer::clear() {
    labels_.clear();
    for (int idx : dirty_cells_) {
        char_z_buffer_[idx] = 1000.0f;
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

void RvizRenderer::plot(int x, int y, float z, ftxui::Color /*color*/, float alpha) {
    // Basic plot with default white color if only Color is provided
    plot(x, y, z, 255, 255, 255, alpha);
}

void RvizRenderer::plot(int x, int y, float z, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    float final_alpha = alpha * global_alpha_;
    if (final_alpha < 0.1f) return;
    int idx = y * width_ + x;
    if (z < z_buffer_[idx]) {
        z_buffer_[idx] = z;
        r_buffer_[idx] = r;
        g_buffer_[idx] = g;
        b_buffer_[idx] = b;
        dot_alphas_[idx] = final_alpha;
        int cx = x / 2, cy = y / 4;
        int cw = width_ / 2;
        int c_idx = cy * cw + cx;
        if (z < char_z_buffer_[c_idx]) {
            if (char_z_buffer_[c_idx] >= 1000.0f) dirty_cells_.push_back(c_idx);
            char_z_buffer_[c_idx] = z;
            char_colors_[c_idx] = ftxui::Color::RGB(static_cast<uint8_t>(r * final_alpha), 
                                                   static_cast<uint8_t>(g * final_alpha), 
                                                   static_cast<uint8_t>(b * final_alpha));
        }
    }
}

void RvizRenderer::draw_point(float x, float y, float z, ftxui::Color /*color*/, float alpha) {
    draw_point(x, y, z, 255, 255, 255, alpha);
}

void RvizRenderer::draw_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    int sx, sy; float sz;
    if (project(x, y, z, sx, sy, sz)) plot(sx, sy, sz, r, g, b, alpha);
}

void RvizRenderer::draw_line(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color /*color*/, float alpha) {
    draw_line(x1, y1, z1, x2, y2, z2, 255, 255, 255, alpha);
}

void RvizRenderer::draw_line(float x1, float y1, float z1, float x2, float y2, float z2, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    int sx1, sy1, sx2, sy2; float sz1, sz2;
    bool p1 = project(x1, y1, z1, sx1, sy1, sz1);
    bool p2 = project(x2, y2, z2, sx2, sy2, sz2);
    if (!p1 && !p2) return;

    if (p1 && p2) {
        int dx = std::abs(sx2 - sx1), dy = std::abs(sy2 - sy1);
        int sx = (sx1 < sx2) ? 1 : -1, sy = (sy1 < sy2) ? 1 : -1;
        int err = dx - dy;
        int x = sx1, y = sy1;
        while (true) {
            float t = (dx > dy) ? (float)std::abs(x - sx1) / (dx+1e-6f) : (float)std::abs(y - sy1) / (dy+1e-6f);
            plot(x, y, sz1 + (sz2 - sz1) * t, r, g, b, alpha);
            if (x == sx2 && y == sy2) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
        }
    } else {
        float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
        for (int i = 0; i <= 50; ++i) {
            float t = (float)i / 50.0f;
            draw_point(x1 + dx * t, y1 + dy * t, z1 + dz * t, r, g, b, alpha);
        }
    }
}

void RvizRenderer::draw_box(float x, float y, float z, float sx, float sy, float sz, ftxui::Color /*color*/, float alpha) {
    draw_box(x, y, z, sx, sy, sz, 255, 255, 255, alpha);
}

void RvizRenderer::draw_box(float x, float y, float z, float sx, float sy, float sz, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    float hx = sx/2, hy = sy/2, hz = sz/2;
    for (float dx : {-hx, hx}) {
        for (float dy : {-hy, hy}) draw_line(x+dx, y+dy, z-hz, x+dx, y+dy, z+hz, r, g, b, alpha);
        for (float dz : {-hz, hz}) draw_line(x+dx, y-hy, z+dz, x+dx, y+hy, z+dz, r, g, b, alpha);
    }
    for (float dy : {-hy, hy}) {
        for (float dz : {-hz, hz}) draw_line(x-hx, y+dy, z+dz, x+hx, y+dy, z+dz, r, g, b, alpha);
    }
}

void RvizRenderer::draw_sphere(float x, float y, float z, float radius, ftxui::Color /*color*/, float alpha) {
    draw_sphere(x, y, z, radius, 255, 255, 255, alpha);
}

void RvizRenderer::draw_sphere(float x, float y, float z, float radius, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    const int segments = 8;
    for (int i = 0; i <= segments; ++i) {
        float lat0 = M_PI * (-0.5f + (float)(i - 1) / segments), z0 = std::sin(lat0) * radius, r0 = std::cos(lat0) * radius;
        float lat1 = M_PI * (-0.5f + (float)i / segments), z1 = std::sin(lat1) * radius, r1 = std::cos(lat1) * radius;
        for (int j = 0; j <= segments; ++j) {
            float lng0 = 2 * M_PI * (float)(j - 1) / segments, x0 = std::cos(lng0), y0 = std::sin(lng0);
            float lng1 = 2 * M_PI * (float)j / segments, x1 = std::cos(lng1), y1 = std::sin(lng1);
            draw_line(x + r1 * x0, y + r1 * y0, z + z1, x + r1 * x1, y + r1 * y1, z + z1, r, g, b, alpha);
            draw_line(x + r1 * x1, y + r1 * y1, z + z1, x + r0 * x1, y + r0 * y1, z + z0, r, g, b, alpha);
        }
    }
}

void RvizRenderer::draw_tile(float x, float y, float z, float sx, float sy, ftxui::Color /*color*/, float alpha) {
    draw_tile(x, y, z, sx, sy, 255, 255, 255, alpha);
}

void RvizRenderer::draw_tile(float x, float y, float z, float sx, float sy, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    float hx = sx/2, hy = sy/2;
    draw_line(x-hx, y-hy, z, x+hx, y-hy, z, r, g, b, alpha);
    draw_line(x+hx, y-hy, z, x+hx, y+hy, z, r, g, b, alpha);
    draw_line(x+hx, y+hy, z, x-hx, y+hy, z, r, g, b, alpha);
    draw_line(x-hx, y+hy, z, x-hx, y-hy, z, r, g, b, alpha);
}

void RvizRenderer::draw_arrow(float x1, float y1, float z1, float x2, float y2, float z2, ftxui::Color /*color*/, float alpha, float head_scale) {
    draw_arrow(x1, y1, z1, x2, y2, z2, 255, 255, 255, alpha, head_scale);
}

void RvizRenderer::draw_arrow(float x1, float y1, float z1, float x2, float y2, float z2, uint8_t r, uint8_t g, uint8_t b, float alpha, float head_scale) {
    draw_line(x1, y1, z1, x2, y2, z2, r, g, b, alpha);
    float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    float len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 1e-6) return;
    dx /= len; dy /= len; dz /= len;
    float head_len = 0.2f * head_scale, head_width = 0.1f * head_scale;
    float ux = (std::abs(dz) < 0.9f) ? 0 : 0, uy = (std::abs(dz) < 0.9f) ? 0 : 1, uz = (std::abs(dz) < 0.9f) ? 1 : 0;
    if (std::abs(dz) < 0.9f) { ux = -dy; uy = dx; uz = 0; } else { ux = 0; uy = -dz; uz = dy; }
    float vlen = std::sqrt(ux*ux + uy*uy + uz*uz); ux /= vlen; uy /= vlen; uz /= vlen;
    float vx = dy*uz - dz*uy, vy = dz*ux - dx*uz, vz = dx*uy - dy*ux;
    for (int i = 0; i < 4; ++i) {
        float angle = i * M_PI / 2.0f;
        float cx = std::cos(angle) * head_width, cy = std::sin(angle) * head_width;
        float px = x2 - dx * head_len + ux * cx + vx * cy;
        float py = y2 - dy * head_len + uy * cx + vy * cy;
        float pz = z2 - dz * head_len + uz * cx + vz * cy;
        draw_line(x2, y2, z2, px, py, pz, r, g, b, alpha);
    }
}
void RvizRenderer::draw_circle(float x, float y, float z, float radius, ftxui::Color /*color*/, float alpha) {
    draw_circle(x, y, z, radius, 255, 255, 255, alpha);
}

void RvizRenderer::draw_circle(float x, float y, float z, float radius, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    const int segments = 16;
    for (int i = 0; i < segments; ++i) {
        float a1 = 2.0f * M_PI * i / segments;
        float a2 = 2.0f * M_PI * (i + 1) / segments;
        draw_line(x + std::cos(a1) * radius, y + std::sin(a1) * radius, z, x + std::cos(a2) * radius, y + std::sin(a2) * radius, z, r, g, b, alpha);
    }
}


void RvizRenderer::finish(ftxui::Canvas& canvas) {
    int cw = width_ / 2;
    for (int c_idx : dirty_cells_) {
        int cx = c_idx % cw, cy = c_idx / cw;
        for (int dy = 0; dy < 4; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                int px = cx * 2 + dx, py = cy * 4 + dy;
                int p_idx = py * width_ + px;
                if (z_buffer_[p_idx] < 1000.0f) {
                    float a = dot_alphas_[p_idx];
                    uint8_t r = static_cast<uint8_t>(r_buffer_[p_idx] * a);
                    uint8_t g = static_cast<uint8_t>(g_buffer_[p_idx] * a);
                    uint8_t b = static_cast<uint8_t>(b_buffer_[p_idx] * a);
                    canvas.DrawPoint(px, py, true, ftxui::Color::RGB(r, g, b));
                }
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
