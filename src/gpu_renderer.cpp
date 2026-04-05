#include "terminal_rviz/gpu_renderer.hpp"

#ifdef USE_GPU
#define CL_TARGET_OPENCL_VERSION 200
#include <iostream>

namespace terminal_rviz {

const char* KERNEL_SOURCE = R"(
typedef struct {
    float z;
    unsigned char r, g, b, padding;
    float alpha;
} Dot;

void atomic_min_float(volatile global float* addr, float val) {
    union {
        float f;
        int i;
    } old, next, current;

    current.f = *addr;
    while (val < current.f) {
        old = current;
        next.f = val;
        current.i = atomic_cmpxchg((volatile global int*)addr, old.i, next.i);
        if (current.i == old.i) break;
    }
}

kernel void render_points(
    global const float* points,
    global const unsigned char* colors,
    uint num_points,
    float m00, float m01, float m02, float m03,
    float m10, float m11, float m12, float m13,
    float m20, float m21, float m22, float m23,
    float zoom,
    int width, int height,
    float global_alpha,
    global Dot* dot_buffer
) {
    uint i = get_global_id(0);
    if (i >= num_points) return;

    float px = points[i*3];
    float py = points[i*3+1];
    float pz = points[i*3+2];

    float sz = m20 * px + m21 * py + m22 * pz + m23;
    if (sz <= 0.1f) return;

    float screen_x = m00 * px + m01 * py + m02 * pz + m03;
    float screen_y = m10 * px + m11 * py + m12 * pz + m13;

    float z_inv = zoom / sz;
    int sx = (width / 2) + (int)(screen_x * z_inv);
    int sy = (height / 2) + (int)(screen_y * z_inv);

    if (sx < 0 || sx >= width || sy < 0 || sy >= height) return;

    int idx = sy * width + sx;
    
    float old_z = dot_buffer[idx].z;
    if (sz < old_z) {
        atomic_min_float(&(dot_buffer[idx].z), sz);
        if (dot_buffer[idx].z == sz) {
            dot_buffer[idx].r = colors[i*4];
            dot_buffer[idx].g = colors[i*4+1];
            dot_buffer[idx].b = colors[i*4+2];
            float p_alpha = (float)colors[i*4+3] / 255.0f;
            dot_buffer[idx].alpha = global_alpha * p_alpha;
        }
    }
}

kernel void render_lines(
    global const float* lines, // x1,y1,z1, x2,y2,z2 triplets
    global const unsigned char* colors,
    uint num_lines,
    float m00, float m01, float m02, float m03,
    float m10, float m11, float m12, float m13,
    float m20, float m21, float m22, float m23,
    float zoom,
    int width, int height,
    float global_alpha,
    global Dot* dot_buffer
) {
    uint i = get_global_id(0);
    if (i >= num_lines) return;

    float x1 = lines[i*6], y1 = lines[i*6+1], z1 = lines[i*6+2];
    float x2 = lines[i*6+3], y2 = lines[i*6+4], z2 = lines[i*6+5];

    float sz1 = m20 * x1 + m21 * y1 + m22 * z1 + m23;
    float sz2 = m20 * x2 + m21 * y2 + m22 * z2 + m23;

    if (sz1 <= 0.1f && sz2 <= 0.1f) return;

    float sx1_f = m00 * x1 + m01 * y1 + m02 * z1 + m03;
    float sy1_f = m10 * x1 + m11 * y1 + m12 * z1 + m13;
    float sx2_f = m00 * x2 + m01 * y2 + m02 * z2 + m03;
    float sy2_f = m10 * x2 + m11 * y2 + m12 * z2 + m13;

    float z_inv1 = zoom / max(sz1, 0.1f);
    float z_inv2 = zoom / max(sz2, 0.1f);

    int sx1 = (width / 2) + (int)(sx1_f * z_inv1);
    int sy1 = (height / 2) + (int)(sy1_f * z_inv1);
    int sx2 = (width / 2) + (int)(sx2_f * z_inv2);
    int sy2 = (height / 2) + (int)(sy2_f * z_inv2);

    // Simple Bresenham-like line drawing
    int dx = abs(sx2 - sx1), dy = abs(sy2 - sy1);
    int sx = (sx1 < sx2) ? 1 : -1, sy = (sy1 < sy2) ? 1 : -1;
    int err = dx - dy;
    int x = sx1, y = sy1;

    uchar r = colors[i*4], g = colors[i*4+1], b = colors[i*4+2];
    float p_alpha = (float)colors[i*4+3] / 255.0f;
    float final_alpha = global_alpha * p_alpha;

    for (int step = 0; step < 2000; ++step) { // Cap steps to prevent infinite loops
        if (x >= 0 && x < width && y >= 0 && y < height) {
            float t = (dx > dy) ? (float)abs(x - sx1) / (float)(dx + 1) : (float)abs(y - sy1) / (float)(dy + 1);
            float cur_z = sz1 + (sz2 - sz1) * t;
            int idx = y * width + x;
            if (cur_z < dot_buffer[idx].z) {
                atomic_min_float(&(dot_buffer[idx].z), cur_z);
                if (dot_buffer[idx].z == cur_z) {
                    dot_buffer[idx].r = r;
                    dot_buffer[idx].g = g;
                    dot_buffer[idx].b = b;
                    dot_buffer[idx].alpha = final_alpha;
                }
            }
        }

        if (x == sx2 && y == sy2) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
}

kernel void clear_buffer(global Dot* dot_buffer, uint size) {
    uint i = get_global_id(0);
    if (i >= size) return;
    dot_buffer[i].z = 1000.0f;
    dot_buffer[i].r = 255;
    dot_buffer[i].g = 255;
    dot_buffer[i].b = 255;
    dot_buffer[i].alpha = 1.0f;
}
)";

GpuRvizRenderer::GpuRvizRenderer() : buf_dots_(nullptr), initialized_(false) {}

GpuRvizRenderer::~GpuRvizRenderer() {
    if (initialized_) {
        if (buf_dots_) clReleaseMemObject(buf_dots_);
        clReleaseKernel(kernel_points_);
        clReleaseKernel(kernel_lines_);
        clReleaseProgram(program_);
        clReleaseCommandQueue(queue_);
        clReleaseContext(context_);
    }
}

bool GpuRvizRenderer::initialize() {
    cl_int err;
    err = clGetPlatformIDs(1, &platform_, NULL);
    if (err != CL_SUCCESS) return false;

    err = clGetDeviceIDs(platform_, CL_DEVICE_TYPE_GPU, 1, &device_, NULL);
    if (err != CL_SUCCESS) {
        err = clGetDeviceIDs(platform_, CL_DEVICE_TYPE_ALL, 1, &device_, NULL);
        if (err != CL_SUCCESS) return false;
    }

    context_ = clCreateContext(NULL, 1, &device_, NULL, NULL, &err);
    if (err != CL_SUCCESS) return false;

    queue_ = clCreateCommandQueue(context_, device_, 0, &err);
    if (err != CL_SUCCESS) return false;

    program_ = clCreateProgramWithSource(context_, 1, &KERNEL_SOURCE, NULL, &err);
    err = clBuildProgram(program_, 1, &device_, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {
        // Print build log on failure
        size_t log_size;
        clGetProgramBuildInfo(program_, device_, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);
        std::vector<char> log(log_size);
        clGetProgramBuildInfo(program_, device_, CL_PROGRAM_BUILD_LOG, log_size, log.data(), NULL);
        std::cerr << "OpenCL Build Error:\n" << log.data() << std::endl;
        return false;
    }

    kernel_points_ = clCreateKernel(program_, "render_points", &err);
    if (err != CL_SUCCESS) return false;

    kernel_lines_ = clCreateKernel(program_, "render_lines", &err);
    if (err != CL_SUCCESS) return false;

    initialized_ = true;
    return true;
}

void GpuRvizRenderer::set_size(int width, int height) {
    if (width_ == width && height_ == height) return;
    width_ = width; height_ = height;
    if (buf_dots_) clReleaseMemObject(buf_dots_);
    
    cl_int err;
    buf_dots_ = clCreateBuffer(context_, CL_MEM_READ_WRITE, width_ * height_ * sizeof(RvizRenderer::Dot), NULL, &err);
    clear();
}

void GpuRvizRenderer::clear() {
    if (!initialized_ || !buf_dots_) return;
    cl_int err;
    cl_kernel k = clCreateKernel(program_, "clear_buffer", &err);
    if (err != CL_SUCCESS) return;

    uint size = width_ * height_;
    clSetKernelArg(k, 0, sizeof(cl_mem), &buf_dots_);
    clSetKernelArg(k, 1, sizeof(uint), &size);
    size_t global_size = size;
    clEnqueueNDRangeKernel(queue_, k, 1, NULL, &global_size, NULL, 0, NULL, NULL);
    clFinish(queue_);
    clReleaseKernel(k);
}

void GpuRvizRenderer::render_points(const std::vector<float>& points, 
                                  const std::vector<uint8_t>& colors, 
                                  const RvizRenderer::Projector& projector,
                                  float alpha) {
    if (!initialized_ || !buf_dots_ || points.empty()) return;

    cl_int err;
    uint num_points = points.size() / 3;
    cl_mem buf_pts = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, points.size() * sizeof(float), (void*)points.data(), &err);
    cl_mem buf_cols = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, colors.size() * sizeof(uint8_t), (void*)colors.data(), &err);

    clSetKernelArg(kernel_points_, 0, sizeof(cl_mem), &buf_pts);
    clSetKernelArg(kernel_points_, 1, sizeof(cl_mem), &buf_cols);
    clSetKernelArg(kernel_points_, 2, sizeof(uint), &num_points);
    clSetKernelArg(kernel_points_, 3, sizeof(float), &projector.m[0][0]);
    clSetKernelArg(kernel_points_, 4, sizeof(float), &projector.m[0][1]);
    clSetKernelArg(kernel_points_, 5, sizeof(float), &projector.m[0][2]);
    clSetKernelArg(kernel_points_, 6, sizeof(float), &projector.m[0][3]);
    clSetKernelArg(kernel_points_, 7, sizeof(float), &projector.m[1][0]);
    clSetKernelArg(kernel_points_, 8, sizeof(float), &projector.m[1][1]);
    clSetKernelArg(kernel_points_, 9, sizeof(float), &projector.m[1][2]);
    clSetKernelArg(kernel_points_, 10, sizeof(float), &projector.m[1][3]);
    clSetKernelArg(kernel_points_, 11, sizeof(float), &projector.m[2][0]);
    clSetKernelArg(kernel_points_, 12, sizeof(float), &projector.m[2][1]);
    clSetKernelArg(kernel_points_, 13, sizeof(float), &projector.m[2][2]);
    clSetKernelArg(kernel_points_, 14, sizeof(float), &projector.m[2][3]);
    clSetKernelArg(kernel_points_, 15, sizeof(float), &projector.zoom);
    clSetKernelArg(kernel_points_, 16, sizeof(int), &width_);
    clSetKernelArg(kernel_points_, 17, sizeof(int), &height_);
    clSetKernelArg(kernel_points_, 18, sizeof(float), &alpha);
    clSetKernelArg(kernel_points_, 19, sizeof(cl_mem), &buf_dots_);

    size_t global_size = num_points;
    clEnqueueNDRangeKernel(queue_, kernel_points_, 1, NULL, &global_size, NULL, 0, NULL, NULL);
    clFinish(queue_);

    clReleaseMemObject(buf_pts);
    clReleaseMemObject(buf_cols);
}

void GpuRvizRenderer::render_lines(const std::vector<float>& lines, 
                                 const std::vector<uint8_t>& colors, 
                                 const RvizRenderer::Projector& projector,
                                 float alpha) {
    if (!initialized_ || !buf_dots_ || lines.empty()) return;

    cl_int err;
    uint num_lines = lines.size() / 6;
    cl_mem buf_lines = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, lines.size() * sizeof(float), (void*)lines.data(), &err);
    cl_mem buf_cols = clCreateBuffer(context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, colors.size() * sizeof(uint8_t), (void*)colors.data(), &err);

    clSetKernelArg(kernel_lines_, 0, sizeof(cl_mem), &buf_lines);
    clSetKernelArg(kernel_lines_, 1, sizeof(cl_mem), &buf_cols);
    clSetKernelArg(kernel_lines_, 2, sizeof(uint), &num_lines);
    clSetKernelArg(kernel_lines_, 3, sizeof(float), &projector.m[0][0]);
    clSetKernelArg(kernel_lines_, 4, sizeof(float), &projector.m[0][1]);
    clSetKernelArg(kernel_lines_, 5, sizeof(float), &projector.m[0][2]);
    clSetKernelArg(kernel_lines_, 6, sizeof(float), &projector.m[0][3]);
    clSetKernelArg(kernel_lines_, 7, sizeof(float), &projector.m[1][0]);
    clSetKernelArg(kernel_lines_, 8, sizeof(float), &projector.m[1][1]);
    clSetKernelArg(kernel_lines_, 9, sizeof(float), &projector.m[1][2]);
    clSetKernelArg(kernel_lines_, 10, sizeof(float), &projector.m[1][3]);
    clSetKernelArg(kernel_lines_, 11, sizeof(float), &projector.m[2][0]);
    clSetKernelArg(kernel_lines_, 12, sizeof(float), &projector.m[2][1]);
    clSetKernelArg(kernel_lines_, 13, sizeof(float), &projector.m[2][2]);
    clSetKernelArg(kernel_lines_, 14, sizeof(float), &projector.m[2][3]);
    clSetKernelArg(kernel_lines_, 15, sizeof(float), &projector.zoom);
    clSetKernelArg(kernel_lines_, 16, sizeof(int), &width_);
    clSetKernelArg(kernel_lines_, 17, sizeof(int), &height_);
    clSetKernelArg(kernel_lines_, 18, sizeof(float), &alpha);
    clSetKernelArg(kernel_lines_, 19, sizeof(cl_mem), &buf_dots_);

    size_t global_size = num_lines;
    clEnqueueNDRangeKernel(queue_, kernel_lines_, 1, NULL, &global_size, NULL, 0, NULL, NULL);
    clFinish(queue_);

    clReleaseMemObject(buf_lines);
    clReleaseMemObject(buf_cols);
}

void GpuRvizRenderer::download(std::vector<RvizRenderer::Dot>& dot_buffer) {
    if (!initialized_ || !buf_dots_) return;
    clEnqueueReadBuffer(queue_, buf_dots_, CL_TRUE, 0, width_ * height_ * sizeof(RvizRenderer::Dot), dot_buffer.data(), 0, NULL, NULL);
}

} // namespace terminal_rviz

#endif // USE_GPU
