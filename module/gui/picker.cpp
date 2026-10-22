#include <module/gui/picker.h>
#include <module/renderer/framebuffer_object.h>
#include <module/renderer/opengl_error.h>


namespace scan3d {


    FramebufferObject *Picker::fbo_ = 0;


    Picker::Picker(const Camera *cam)
            : camera_(cam), use_gpu_if_supported_(true) {
    }


    Picker::~Picker() {
        delete fbo_;
        fbo_ = nullptr;
    }


    void Picker::setup_framebuffer(int width, int height) {
        // prepare a frame buffer object (fbo), I will do offscreen rendering to the new fbo
        if (!fbo_) {
            fbo_ = new FramebufferObject(width, height, 0); scan3d_debug_log_gl_error; scan3d_debug_log_frame_buffer_error;
            fbo_->add_color_buffer(); scan3d_debug_log_gl_error; scan3d_debug_log_frame_buffer_error;
            fbo_->add_depth_buffer(); scan3d_debug_log_gl_error; scan3d_debug_log_frame_buffer_error;

#if 0
            fbo_->print_attachments();
            fbo_->print_draw_buffers();
            fbo_->print_read_buffer();
#endif
        }
        fbo_->ensure_size(width, height); scan3d_debug_log_gl_error; scan3d_debug_log_frame_buffer_error;
    }


    void Picker::screen_to_opengl(int x, int y, int &gl_x, int &gl_y, int width, int height) const {
        float dpi_scaling_x = width / static_cast<float>(camera()->screenWidth());
        float dpi_scaling_y = height / static_cast<float>(camera()->screenHeight());

        gl_x = static_cast<int>(dpi_scaling_x * x);
        gl_y = static_cast<int>(dpi_scaling_y * (camera()->screenHeight() - 1 - y));
    }

}