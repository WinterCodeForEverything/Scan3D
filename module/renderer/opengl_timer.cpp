#include <module/renderer/opengl_timer.h>

#include <module/renderer/opengl.h>
#include <module/renderer/opengl_error.h>

#include <cassert>


namespace scan3d {

    OpenGLTimer::OpenGLTimer(bool start_timing /* = true*/)
        : query_id_(0)
        , running_(false)
    {
        glGenQueries(1, &query_id_); scan3d_debug_log_gl_error;

        if (start_timing)
            start();
    }


    OpenGLTimer::~OpenGLTimer() {
        if (query_id_) {
            glDeleteQueries(1, &query_id_); scan3d_debug_log_gl_error;
        }
    }


    void OpenGLTimer::start() {
        assert(!running_);
        glBeginQuery(GL_TIME_ELAPSED, query_id_);	scan3d_debug_log_gl_error;
        running_ = true;
    }


    void OpenGLTimer::stop() {
        assert(running_);
        glEndQuery(GL_TIME_ELAPSED);
        running_ = false;
    }

    double OpenGLTimer::time() {
        if (running_)
            stop();

        // check and wait until the result is available
        GLuint queryReady = 0;
        do {
            // check whether the passed samples counter is immediately available. If a delay
            // would occur waiting for the query result, GL_FALSE is returned. Otherwise,
            // GL_TRUE is returned, which also indicates that the results of all previous
            // queries are available as well.
            glGetQueryObjectuiv(query_id_, GL_QUERY_RESULT_AVAILABLE, &queryReady);
        } while (!queryReady);

        GLuint query_time = 0;
        glGetQueryObjectuiv(query_id_, GL_QUERY_RESULT, &query_time);   scan3d_debug_log_gl_error

        return static_cast<double>(query_time) / 1000000.0; // in milliseconds
    }

}
