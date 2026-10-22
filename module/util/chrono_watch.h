#ifndef SCAN3D_UTIL_CHRONO_WATCH_H
#define SCAN3D_UTIL_CHRONO_WATCH_H


#include <chrono>
#include <ostream>
#include <string>

namespace scan3d {

    /**
    * usage example:
    *   {
    *      ChronoWatch t;
    *	   t.start();
    *
    *      // do task_1 ...
    *      LOG(INFO) << "task_1 done. Time: " << t.time_string();
    *
    *	   t.reset(), t.start();
    *      // do task_2 ...
    *      LOG(INFO) << "task_1 done. Time: " << t.time_string();
    *   }
    */

    // Liangliang:
    // std::chrono::high_resolution_clock has disappointing accuracy on windows.
    // On windows, you should use the WinAPI QueryPerformanceFrequency().
    // If you care much about accuracy, use the StopWatch instead.
    class ChronoWatch
    {
    public:
        ChronoWatch();
        ~ChronoWatch() {}

        void start();
        void restart();

        void pause();
        void resume();

        void reset();

        float elapsed_milliseconds(int num_digits = 1) const;
        float elapsed_seconds(int num_digits = 1) const;
        float elapsed_minutes(int num_digits = 1) const;
        float elapsed_hours(int num_digits = 1) const;

        void print_seconds(std::ostream& output) const;
        void print_minutes(std::ostream& output) const;
        void print_hours(std::ostream& output) const;

        std::string time_string(int num_digits = 1) const;

    private:
        std::size_t duration_count() const;

    private:
        bool started_;
        std::chrono::high_resolution_clock::time_point start_time_;

        bool paused_;
        std::chrono::high_resolution_clock::time_point pause_time_;
    };

}

#endif  // SCAN3D_UTIL_CHRONO_WATCH_H
