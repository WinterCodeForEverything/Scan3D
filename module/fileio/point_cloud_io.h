#ifndef SCAN3D_FILEIO_POINT_CLOUD_IO_H
#define SCAN3D_FILEIO_POINT_CLOUD_IO_H

#include <iostream>
#include <vector>


namespace scan3d {


	class PointCloud;

    // TODO: add a general purpose file format 'map' capable of storing
    //       point clouds, graphs, and meshes with arbitrary types of
    //       properties (similar to PLY, each property is read/written
    //       as a whole. Check PLY and POLY formats as reference.
    //       When this is done, I don't need BIN and POLY any more).

	class PointCloudIO
	{
	public:
        // return nullptr if failed
		static PointCloud* load(const std::string& file_name);

		// save the cloud to a file. return false if failed.
		static bool	save(const std::string& file_name, const PointCloud* point_set);
	};


	namespace io {
		// three blocks storing points, colors (optional), and normals (optional)
		bool load_bin(const std::string& file_name, PointCloud* cloud);
		bool save_bin(const std::string& file_name, const PointCloud* cloud);

		// each line with only point: (x, y, z)
		bool load_xyz(const std::string& file_name, PointCloud* cloud);
		bool save_xyz(const std::string& file_name, const PointCloud* cloud);
		bool load_bxyz(const std::string& file_name, PointCloud* cloud);
		bool save_bxyz(const std::string& file_name, const PointCloud* cloud);

		bool load_ply(const std::string& file_name, PointCloud* cloud);
		bool save_ply(const std::string& file_name, const PointCloud* cloud, bool binary = true);

		bool load_obj(const std::string& file_name, PointCloud* cloud);
		bool save_obj(const std::string& file_name, const PointCloud* cloud);

		//bool load_3mf(const std::string& file_name, PointCloud* cloud);

		// Read/Write both las and laz formats. Internally it uses the LASlib
        // of martin.isenburg@rapidlasso.com, see http://rapidlasso.com
		bool load_las(const std::string& file_name, PointCloud* cloud);
		bool save_las(const std::string& file_name, const PointCloud* cloud);

	};


} // namespace scan3d


#endif  // SCAN3D_FILEIO_POINT_CLOUD_IO_H
