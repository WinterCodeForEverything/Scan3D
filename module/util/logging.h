#ifndef SCAN3D_UTIL_LOGGING_H
#define SCAN3D_UTIL_LOGGING_H


/* Make it easier for including the Google glog header file.
 *
 * Always include this file, i.e.,
 *      #include <module/util/logging.h>
 * instead of including any other header files, because the third parties may change.
 */

#include <3rd_party/glog/glog/logging.h>
#include <3rd_party/glog/glog/stl_logging.h>    // to allow logging STL containers


/** If failed, a fatal error will be logged */
#define CHECK_TRUE(val)     CHECK_EQ((val), true)
#define CHECK_FALSE(val)    CHECK_EQ((val), false)

namespace scan3d
{

	namespace logging
	{

		/**
		 * @brief Initializes Google's logging library. The log will be writen to log files.
		 * @param severity_dependent True to create a log file for each severity, otherwise a single log file each run.
		 * @param log_dir The directory where the log files will be created. If it is empty (default value), the log
		 * 		  files will be created in the "logs" directory next the executable file.
		 * @param stderr_threshold Log messages at a level >= this flag are automatically sent to standard error
		 *        (stderr) in addition to log files. Acceptable values are
		 *          - google::GLOG_INFO
		 *          - google::GLOG_WARNING
		 *          - google::GLOG_ERROR
		 * @note Initialization of Google's logging library is not mandatory. Logging before initialize() will be
		 * 		 written to stderr.
		 */
		void initialize(bool severity_dependent = false, const std::string& log_dir = "", int stderr_threshold = google::GLOG_WARNING);

	}

}

#endif // SCAN3D_UTIL_LOGGING_H
