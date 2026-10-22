#ifndef SCAN3D_UTIL_STOP_WATCH_H
#define SCAN3D_UTIL_STOP_WATCH_H

#include <string>

#ifdef _WIN32
#include <cstdint>
#else
#include <sys/time.h>
#endif // _WIN32


namespace scan3d {

    /* High resolution stop watch/timer.
     * This timer is able to measure the elapsed time with 1 micro-second accuracy
     * in both Windows, Linux, and Unix system.
     *
     * Usage example:
     *
     *      StopWatch w ;
     *      // do task_1 ...
     *      LOG(INFO) << "task_1 done. Time: " << w.time_string() << " seconds";
     *	   w.start();
     *      // do task_2 ...
     *      LOG(INFO) << "task_2 done. Time: " << w.time_string() << " seconds";
     *
     */

    class StopWatch {
    public:
        StopWatch(); // the watch will automatically start in construction
        ~StopWatch();

        void start();

        void restart(); // the same as start()

        // returns user elapsed time since the construction / start in seconds.
        double elapsed_seconds(int num_digits = 1) const;

        std::string time_string(int num_digits = 1) const;

    private:
        double seconds() const;

#ifdef _WIN32
        int64_t  freq_;
        int64_t  start_count_;
#else
        timeval start_time_;
#endif

    };

} // namespace scan3d


#endif  // SCAN3D_UTIL_STOP_WATCH_H

