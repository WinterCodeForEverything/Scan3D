#include <module/util/logging.h>
#include <module/util/file_system.h>

#include <3rd_party/glog/src/utilities.h>

#include <iostream>


namespace scan3d
{

	namespace logging
	{

		void initialize(bool severity_dependent, const std::string& log_dir, int stderr_threshold)
		{
			// Ensures the library will not be initialized more than once.
			if (google::glog_internal_namespace_::IsGoogleLoggingInitialized())
				return;

			// use "static" to make sure the string still exist outside this function (glog is multi threaded).
            static std::string app_path = file_system::executable();

			std::string log_path = log_dir;
			if (log_path.empty())
			{
				log_path = app_path;
#ifdef __APPLE__
				// macOS may put the executable file in an application bundle, e.g., "PolyFit.app/Contents/MacOS/PolyFit"
				std::string::size_type pos = log_path.find(".app");
				if (pos != std::string::npos)
					log_path = log_path.substr(0, pos);
#endif
				log_path = file_system::parent_directory(log_path) + "/logs";
			}
            else    // always append a separator in case the user forgot it
                log_path = log_dir + file_system::native_path_separator();

			if (!file_system::is_directory(log_path))
				file_system::create_directory(log_path);
			LOG_IF(ERROR, !file_system::is_directory(log_path)) << "could not create log directory: " << log_path;

			if (severity_dependent) // create a log file for each severity level
				FLAGS_log_dir = log_path;
			else {
			    const std::string& dest = log_path + "/" + file_system::base_name(app_path) + "-";
                google::SetLogDestination(google::GLOG_INFO, dest.c_str());
			}

			FLAGS_stderrthreshold = stderr_threshold;
			FLAGS_colorlogtostderr = true;
			google::InitGoogleLogging(app_path.c_str());

//            auto failure_dump = [](const char* data, int size) -> void {
//                LOG(ERROR) << std::string(data, size)
//                << "\n-----------------------------------------\n"
//                   "Oh, Scan3D crashed. Please contact me ( jianwei.xu@gmail.com) for more information.";
//            };
//            google::InstallFailureWriter(failure_dump);

			DLOG(INFO) << "logger initialized";
			DLOG(INFO) << "executable path: " << file_system::executable_directory();
			DLOG(INFO) << "current working dir: " << file_system::current_working_directory();
		}
	}

}

