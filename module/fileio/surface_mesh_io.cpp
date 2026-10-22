#include <module/fileio/surface_mesh_io.h>

#include <clocale>

#include <module/core/surface_mesh.h>
#include <module/util/file_system.h>
#include <module/util/stop_watch.h>
#include <module/util/logging.h>


namespace scan3d {


	SurfaceMesh* SurfaceMeshIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

		SurfaceMesh* mesh = new SurfaceMesh;
		mesh->set_name(file_name);

		StopWatch w;
		bool success = false;

        const std::string& ext = file_system::extension(file_name, true);
        if (ext == "ply")
            success = io::load_ply(file_name, mesh);
        else if (ext == "poly")
            success = io::load_poly(file_name, mesh);
        else if (ext == "obj")
			success = io::load_obj(file_name, mesh);
		else if (ext == "off")
			success = io::load_off(file_name, mesh);
		else if (ext == "stl")
			success = io::load_stl(file_name, mesh);
		else if (ext == "3mf")
			success = io::load_3mf(file_name, mesh);
        else if (ext.empty()){
            LOG(ERROR) << "unknown file format: no extension" << ext;
            success = false;
        }
		else {
			LOG(ERROR) << "unknown file format: " << ext;
			success = false;
		}

		if (!success || mesh->n_faces() == 0) {
			delete mesh;
			return nullptr;
		}

        if (success)
            LOG(INFO) << "surface mesh loaded ("
                      << "#face: " << mesh->n_faces() << ", "
                      << "#vertex: " << mesh->n_vertices() << ", "
                      << "#edge: " << mesh->n_edges() << "). "
                      << w.time_string();
        else
            LOG(INFO) << "load surface mesh failed";

		return mesh;
	}


	bool SurfaceMeshIO::save(const std::string& file_name, const SurfaceMesh* mesh)
	{
        if (!mesh || mesh->n_faces() == 0) {
			LOG(ERROR) << "surface mesh is null";
			return false;
		}

		StopWatch w;
		bool success = false;

        std::string final_name = file_name;
        const std::string& ext = file_system::extension(file_name, true);

        if (ext == "ply" || ext.empty()) {
            if (ext.empty()) {
                LOG(ERROR) << "no extension specified, default to ply" << ext;
                final_name = final_name + ".ply";
            }
            success = io::save_ply(final_name, mesh, true);
        }
        else if (ext == "poly")
            success = io::save_poly(final_name, mesh);
        else if (ext == "obj")
            success = io::save_obj(final_name, mesh);
        else if (ext == "off")
            success = io::save_off(final_name, mesh);
		else if (ext == "stl")
            success = io::save_stl(final_name, mesh);
		else if (ext == "3mf")
			success = io::save_3mf(final_name, mesh);
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
