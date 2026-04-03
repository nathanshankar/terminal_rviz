#ifndef TERMINAL_RVIZ_GPU_RENDERER_HPP_
#define TERMINAL_RVIZ_GPU_RENDERER_HPP_

#ifdef USE_GPU
#define CL_TARGET_OPENCL_VERSION 200
#include <CL/cl.h>
#include <vector>
#include <string>
#include "terminal_rviz/renderer.hpp"

namespace terminal_rviz {

class GpuRvizRenderer {
public:
    GpuRvizRenderer();
    ~GpuRvizRenderer();

    bool initialize();
    void set_size(int width, int height);
    
    // Uploads a batch of points to the GPU and renders them into the Z-buffer
    void render_points(const std::vector<float>& points, // x,y,z triplets
                      const std::vector<uint8_t>& colors, // r,g,b triplets
                      const RvizRenderer::Projector& projector,
                      float alpha);

    void clear();
    
    // Downloads the full result buffer back to CPU
    void download(std::vector<RvizRenderer::Dot>& dot_buffer);

private:
    cl_platform_id platform_;
    cl_device_id device_;
    cl_context context_;
    cl_command_queue queue_;
    cl_program program_;
    cl_kernel kernel_points_;

    cl_mem buf_dots_;
    int width_ = 0, height_ = 0;
    bool initialized_ = false;
};

} // namespace terminal_rviz

#endif // USE_GPU
#endif // TERMINAL_RVIZ_GPU_RENDERER_HPP_
