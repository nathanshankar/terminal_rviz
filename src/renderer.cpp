#include "terminal_rviz/renderer.hpp"
#include <algorithm>

namespace terminal_rviz {

RvizRenderer::RvizRenderer() {}

void RvizRenderer::set_size(int width, int height) {
    if (width_ != width || height_ != height) {
        width_ = width;
        height_ = height;
        dot_buffer_.assign(width_ * height_, Dot());
        char_z_buffer_.assign((width_ / 2) * (height_ / 4), 1000.0f);
        char_colors_.assign((width_ / 2) * (height_ / 4), ftxui::Color::White);
        is_dirty_.assign((width_ / 2) * (height_ / 4), false);
        dirty_cells_.clear();
    }
}

void RvizRenderer::set_camera(float yaw, float pitch, float roll, float dist, float cx, float cy, float cz) {
    yaw_ = yaw; pitch_ = pitch; roll_ = roll; dist_ = dist; cx_ = cx; cy_ = cy; cz_ = cz;
    
    float s_y = std::sin(yaw_), c_y = std::cos(yaw_);
    float s_p = std::sin(pitch_), c_p = std::cos(pitch_);
    float s_r = std::sin(roll_), c_r = std::cos(roll_);

    // tx: rx=0, ry=0, rz=1
    m00_ = s_y * c_r + c_y * s_p * s_r;
    m10_ = s_y * s_r - c_y * s_p * c_r;
    m20_ = c_y * c_p;

    // ty: rx=-1, ry=0, rz=0
    m01_ = -c_y * c_r + s_y * s_p * s_r;
    m11_ = -c_y * s_r - s_y * s_p * c_r;
    m21_ = s_y * c_p;

    // tz: rx=0, ry=-1, rz=0
    m02_ = c_p * s_r;
    m12_ = -c_p * c_r;
    m22_ = -s_p;
}

void RvizRenderer::clear() {
    labels_.clear();
    int cw = width_ / 2;
    for (int idx : dirty_cells_) {
        char_z_buffer_[idx] = 1000.0f;
        is_dirty_[idx] = false;
        int cx = idx % cw, cy = idx / cw;
        int py_base = cy * 4;
        int px_base = cx * 2;
        for (int dy = 0; dy < 4; ++dy) {
            int row_offset = (py_base + dy) * width_;
            dot_buffer_[row_offset + px_base].z = 1000.0f;
            dot_buffer_[row_offset + px_base + 1].z = 1000.0f;
        }
    }
    dirty_cells_.clear();
}

bool RvizRenderer::project(float dx, float dy, float dz, int& out_sx, int& out_sy, float& out_z) const {
    float tx = dx - cx_, ty = dy - cy_, tz = dz - cz_;
    
    out_z = m20_ * tx + m21_ * ty + m22_ * tz + dist_;
    if (out_z > 0.1f) {
        float x3 = m00_ * tx + m01_ * ty + m02_ * tz;
        float y3 = m10_ * tx + m11_ * ty + m12_ * tz;
        float z_inv = zoom_ / out_z;
        out_sx = (width_ / 2) + static_cast<int>(x3 * z_inv);
        out_sy = (height_ / 2) + static_cast<int>(y3 * z_inv);
        return true;
    }
    return false;
}

void RvizRenderer::plot(int x, int y, float z, ftxui::Color /*color*/, float alpha) {
    plot(x, y, z, 255, 255, 255, alpha);
}

void RvizRenderer::plot(int x, int y, float z, uint8_t r, uint8_t g, uint8_t b, float alpha) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    float final_alpha = alpha * global_alpha_;
    if (final_alpha < 0.1f) return;
    int idx = y * width_ + x;
    Dot& dot = dot_buffer_[idx];
    if (z < dot.z) {
        dot.z = z;
        dot.r = r;
        dot.g = g;
        dot.b = b;
        dot.alpha = final_alpha;
        
        int cx = x / 2, cy = y / 4;
        int cw = width_ / 2;
        int c_idx = cy * cw + cx;
        
        if (z < char_z_buffer_[c_idx]) {
            if (!is_dirty_[c_idx]) {
                dirty_cells_.push_back(c_idx);
                is_dirty_[c_idx] = true;
            }
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
        int py_base = cy * 4;
        int px_base = cx * 2;
        for (int dy = 0; dy < 4; ++dy) {
            int py = py_base + dy;
            int row_offset = py * width_;
            for (int dx = 0; dx < 2; ++dx) {
                int px = px_base + dx;
                const Dot& dot = dot_buffer_[row_offset + px];
                if (dot.z < 1000.0f) {
                    uint8_t r = static_cast<uint8_t>(dot.r * dot.alpha);
                    uint8_t g = static_cast<uint8_t>(dot.g * dot.alpha);
                    uint8_t b = static_cast<uint8_t>(dot.b * dot.alpha);
                    canvas.DrawPoint(px, py, true, ftxui::Color::RGB(r, g, b));
                }
            }
        }
    }
}

RvizRenderer::Projector RvizRenderer::get_projector(const tf2::Transform& world_to_object) const {
    Projector p;
    p.zoom = zoom_;
    p.width = width_;
    p.height = height_;

    // M_view row 0: m00, m01, m02, -(m00*cx + m01*cy + m02*cz)
    // M_view row 1: m10, m11, m12, -(m10*cx + m11*cy + m12*cz)
    // M_view row 2: m20, m21, m22, -(m20*cx + m21*cy + m22*cz) + dist
    
    float v03 = -(m00_ * cx_ + m01_ * cy_ + m02_ * cz_);
    float v13 = -(m10_ * cx_ + m11_ * cy_ + m12_ * cz_);
    float v23 = -(m20_ * cx_ + m21_ * cy_ + m22_ * cz_) + dist_;

    tf2::Matrix3x3 rot = world_to_object.getBasis();
    tf2::Vector3 trans = world_to_object.getOrigin();

    // Combined matrix = M_view * world_to_object
    // [ m00 m01 m02 v03 ]   [ r00 r01 r02 t0 ]
    // [ m10 m11 m12 v13 ] * [ r10 r11 r12 t1 ]
    // [ m20 m21 m22 v23 ]   [ r20 r21 r22 t2 ]
    //                       [  0   0   0   1 ]
    
    for (int i = 0; i < 3; ++i) {
        float m_row_i[3];
        float v_i3;
        if (i == 0) { m_row_i[0] = m00_; m_row_i[1] = m01_; m_row_i[2] = m02_; v_i3 = v03; }
        else if (i == 1) { m_row_i[0] = m10_; m_row_i[1] = m11_; m_row_i[2] = m12_; v_i3 = v13; }
        else { m_row_i[0] = m20_; m_row_i[1] = m21_; m_row_i[2] = m22_; v_i3 = v23; }

        for (int j = 0; j < 3; ++j) {
            p.m[i][j] = m_row_i[0] * rot[0][j] + m_row_i[1] * rot[1][j] + m_row_i[2] * rot[2][j];
        }
        p.m[i][3] = m_row_i[0] * trans.x() + m_row_i[1] * trans.y() + m_row_i[2] * trans.z() + v_i3;
    }
    return p;
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
