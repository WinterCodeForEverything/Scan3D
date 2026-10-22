#ifndef SCAN3D_UTIL_PROGRESS_H
#define SCAN3D_UTIL_PROGRESS_H


#include <string>


namespace scan3d {

    /**
     * The based class of GUI element showing the progress.
     */
    class ProgressClient {
    public:
        ProgressClient();
        virtual ~ProgressClient() {}
        virtual void notify(std::size_t new_value, bool show_text = true, bool update_viewer = true) = 0;
        virtual void cancel();
    };

    //_________________________________________________________

    class ProgressLogger {
    public:
        ProgressLogger(std::size_t max_val = 100, const std::string &task_name = "", bool quiet = false);
        virtual ~ProgressLogger();

        virtual void notify(std::size_t new_value, bool show_text = true, bool update_viewer = true);
        virtual void next(bool update_viewer = true);
        virtual void done() { notify(max_val_); }

        bool is_canceled() const;

        void reset(bool show_text = true) { notify(0, show_text); }
        void reset(std::size_t max_val, bool show_text = true);

    protected:
        virtual void update(bool show_text, bool update_viewer);

    private:
        std::size_t max_val_;
        std::string task_name_;
        std::size_t cur_val_;
        std::size_t cur_percent_;
        bool quiet_;
    };

}   // namespace scan3d


#endif  // SCAN3D_UTIL_PROGRESS_H

