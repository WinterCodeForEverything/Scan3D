#ifndef SCAN3D_FILEIO_MESH_IO_H
#define SCAN3D_FILEIO_MESH_IO_H


#include <string>


namespace scan3d {

	class SurfaceMesh;

    // TODO: add a general purpose file format 'map' capable of storing
    //       point clouds, graphs, and meshes with arbitrary types of
    //       properties (similar to PLY, each property is read/written
    //       as a whole. Check PLY and POLY formats as reference.
    //       When this is done, I don't need BIN and POLY any more).

	class SurfaceMeshIO
	{
	public:
		// return nullptr if failed.
		static SurfaceMesh* load(const std::string& file_name);

		// save the mesh to a file. return false if failed.
		static bool	save(const std::string& file_name, const SurfaceMesh* mesh);
	};

	namespace io {

        bool load_poly(const std::string& file_name, SurfaceMesh* mesh);
        bool save_poly(const std::string& file_name, const SurfaceMesh* mesh);

        bool load_ply(const std::string& file_name, SurfaceMesh* mesh);
        bool save_ply(const std::string& file_name, const SurfaceMesh* mesh, bool binary = true);

		bool load_off(const std::string& file_name, SurfaceMesh* mesh);
		bool save_off(const std::string& file_name, const SurfaceMesh* mesh);

		bool load_obj(const std::string& file_name, SurfaceMesh* mesh);
		bool save_obj(const std::string& file_name, const SurfaceMesh* mesh);

		bool load_stl(const std::string& file_name, SurfaceMesh* mesh);
		bool save_stl(const std::string& file_name, const SurfaceMesh* mesh);

		bool load_3mf(const std::string& file_name, SurfaceMesh* mesh);
		bool save_3mf(const std::string& file_name, const SurfaceMesh* mesh);

	} // namespace io

} // namespace scan3d

#endif // SCAN3D_FILEIO_MESH_IO_H
