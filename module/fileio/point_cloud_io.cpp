#include <module/fileio/point_cloud_io.h>

#include <clocale>

#include <module/fileio/point_cloud_io_vg.h>
#include <module/core/point_cloud.h>
#include <module/util/file_system.h>
#include <module/util/stop_watch.h>


namespace scan3d {


	PointCloud* PointCloudIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

		PointCloud* cloud = new PointCloud;
		cloud->set_name(file_name);

		StopWatch w;
		bool success = false;

        const std::string& ext = file_system::extension(file_name, true);
		if (ext == "ply")
			success = io::load_ply(file_name, cloud);
		else if (ext == "bin")
			success = io::load_bin(file_name, cloud);
		else if (ext == "xyz" || ext == "asc")
			success = io::load_xyz(file_name, cloud);
		else if (ext == "bxyz")
			success = io::load_bxyz(file_name, cloud);
		else if (ext == "las" || ext == "laz")
			success = io::load_las(file_name, cloud);
		else if (ext == "vg")
			success = io::PointCloudIO_vg::load_vg(file_name, cloud);
		else if (ext == "bvg")
			success = io::PointCloudIO_vg::load_bvg(file_name, cloud);
		else if (ext == "obj")
			success = io::load_obj( file_name, cloud );

        else if (ext.empty()){
            LOG(ERROR) << "unknown file format: no extension";
            success = false;
        }
		else {
			LOG(ERROR) << "unknown file format: " << ext;
			success = false;
		}

		if (!success || cloud->n_vertices() == 0) {
			delete cloud;
			return nullptr;
		}

        if (success)
            LOG(INFO) << "point cloud loaded ("
                      << "#vertex: " << cloud->n_vertices() << "). "
                      << w.time_string();
        else
            LOG(INFO) << "load point cloud failed.";

		return cloud;
	}


	bool PointCloudIO::save(const std::string& file_name, const PointCloud* cloud) {
		if (!cloud) {
			LOG(ERROR) << "Point cloud is null";
			return false;
		}

        StopWatch w;
        bool success = false;

        std::string final_name = file_name;
        const std::string& ext = file_system::extension(file_name, true);
        if (ext == "ply" || ext.empty()) {
            if (ext.empty()) {
                LOG(ERROR) << "No extension specified. Default to ply";
                final_name = final_name + ".ply";
            }
            success = io::save_ply(final_name, cloud, true);
        }
		else if (ext == "obj")
			success = io::save_obj(final_name, cloud);
		else if (ext == "bin")
            success = io::save_bin(final_name, cloud);
		else if (ext == "xyz" || ext == "asc" )
            success = io::save_xyz(final_name, cloud);
		else if (ext == "bxyz")
            success = io::save_bxyz(final_name, cloud);
		else if (ext == "las" || ext == "laz")
            success = io::save_las(final_name, cloud);
        else if (ext == "vg")
            success = io::PointCloudIO_vg::save_vg(file_name, cloud);
        else if (ext == "bvg")
            success = io::PointCloudIO_vg::save_bvg(file_name, cloud);
		else {
			LOG(ERROR) << "unknown file format: " << ext;
            success = false;
		}

        if (success) {
            LOG(INFO) << "save model done. " << w.time_string();
            return true;
        }
        else {
            LOG(INFO) << "save model failed";
            return false;
        }

	}

} // namespace scan3d
