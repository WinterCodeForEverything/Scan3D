#ifndef SCAN3D_RENDERER_OPENGL_ERROR_H
#define SCAN3D_RENDERER_OPENGL_ERROR_H

#include <string>
#include <module/util/logging.h>


namespace scan3d {


#define scan3d_gl_error {                                                   \
    opengl::check_gl_error(__FILE__, __FUNCTION__, __LINE__) ;              \
}

#define scan3d_frame_buffer_error {                                         \
    opengl::check_frame_buffer_error(__FILE__, __FUNCTION__, __LINE__) ;    \
}

#define scan3d_log_gl_error {                                               \
    std::string log("");                                                    \
    LOG_IF(ERROR, !opengl::gl_error(log)) << "GL error: \n"                 \
    << "\tfile: " << __FILE__ << "\n"                                       \
    << "\tline: " << __LINE__ << "\n"                                       \
    << "\tfunction: " << __FUNCTION__ << "\n"                               \
    << "\tinfo: " << log;                                                   \
}

#define scan3d_log_frame_buffer_error {                                     \
    std::string log("");                                                    \
    LOG_IF(ERROR, !opengl::frame_buffer_error(log)) << "GL error: \n"       \
    << "\tfile: " << __FILE__ << "\n"                                       \
    << "\tline: " << __LINE__ << "\n"                                       \
    << "\tfunction: " << __FUNCTION__ << "\n"                               \
    << "\tinfo: " << log;                                                   \
}


#ifndef NDEBUG
#define scan3d_debug_gl_error				scan3d_gl_error
#define scan3d_debug_frame_buffer_error		scan3d_frame_buffer_error
#define scan3d_debug_log_gl_error			scan3d_log_gl_error
#define scan3d_debug_log_frame_buffer_error	scan3d_log_frame_buffer_error
#else
#define scan3d_debug_gl_error
#define scan3d_debug_frame_buffer_error
#define scan3d_debug_log_gl_error
#define scan3d_debug_log_frame_buffer_error
#endif


    namespace opengl {

        // Prints the last GL error to the logger.
        // returns false if an error occurred.
        bool check_gl_error(const std::string& file, const std::string& function, int line);
        bool check_frame_buffer_error(const std::string& file, const std::string& function, int line);

        // Check if there was an error. The error message will be stored in "log" if provided.
        // returns false if an error occurred.
        bool gl_error(std::string& log);
        bool frame_buffer_error(std::string& log);

        void setup_gl_debug_callback();

    }

}


#endif  // SCAN3D_RENDERER_OPENGL_ERROR_H


