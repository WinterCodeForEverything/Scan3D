#ifndef SCAN3D_RENDERER_OPENGL_TIMER_H
#define SCAN3D_RENDERER_OPENGL_TIMER_H

namespace scan3d {

    // OpenGLTimer: Accurate timing of GPU operations. Because the GPU is based
    // on an asynchronously processed command stream that is implicitly generated
    // by OpenGL calls, it is not possible to determine the amount of time spent
    // processing a set of OpenGL calls by timing the calls themselves at the
    // application level. Instead, the timing must be done via an asynchronous
    // timer system. This timer are started and stopped at the beginning and end
    // of the OpenGL commands of interest. However, the timings results must be
    // queried at some point later, as the pipeline must complete before
    // the timer values are known. This class implements such a timer that behaves
    // like a CPU timer (see util/stop_watch.h)

    /* Usage:
     *
     *      OpenGLTimer t;
     *      // opengl calls ...
     *      draw();
     *      fprintf(stdout, "\rTime: %2.4f ms    ", t.time());
     */

    class OpenGLTimer
    {
    public:
        // start the new timer by default.
        OpenGLTimer(bool start_timing = true);
        virtual ~OpenGLTimer();

        // Start the timer. The next OpenGL call will be the first timed).
        // This must be called from a thread with the OpenGL context bound.
        void	start();

        // Is the timer running.
        bool	is_running() const { return running_; }

        // Stop the timer (the previous OpenGL call will be the last timed).
        // This must be called from a thread with the OpenGL context bound.
        void    stop();

        // return the GPU time consumed since last start (in milliseconds).
        double	time();

    protected:
        unsigned int query_id_;
        bool         running_;
    };

}


#endif  // SCAN3D_RENDERER_OPENGL_TIMER_H
