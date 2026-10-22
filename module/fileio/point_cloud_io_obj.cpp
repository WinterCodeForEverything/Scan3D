#include <module/fileio/point_cloud_io.h>

#include <unordered_map>

#include <module/core/point_cloud.h>
#include <module/util/file_system.h>
#include <module/util/logging.h>
#include <3rd_party/tinyobjloader/tiny_obj_loader.h>

namespace scan3d {

    namespace io {

        bool load_obj(const std::string &file_name, PointCloud *cloud) {
            if (!cloud) {
                LOG(ERROR) << "null point cloud pointer";
                return false;
            }

            if (!file_system::is_file(file_name)) {
                LOG(ERROR) << "file does not exist: " << file_system::simple_name(file_name);
                return false;
            }

            tinyobj::ObjReaderConfig config;
			config.vertex_color = false;
            tinyobj::ObjReader reader;
            if (!reader.ParseFromFile(file_name, config)) {
                LOG(ERROR) << "failed parsing file: " + file_name + ". " << reader.Error() << reader.Warning();
                return false;
            }

            // --------------------- collect the data ------------------------

			cloud->clear();
            //const std::vector<tinyobj::shape_t> &shapes = reader.GetShapes();
            //if (shapes.empty()) {
            //    LOG(WARNING) << "file contains no shape";
            //    return false;
            //}
            const tinyobj::attrib_t &attrib = reader.GetAttrib();

			// add vertices and attributes
			bool bNormal = false;
			PointCloud::VertexProperty<vec3> normal;
			if ( attrib.normals.size() )
			{
				bNormal = true;
				normal = cloud->vertex_property<vec3>("v:normal");
			}
			for (std::size_t v = 0; v < attrib.vertices.size(); v += 3) {
				auto vertex = cloud->add_vertex(vec3(attrib.vertices.data() + v));
				if( bNormal )
					normal[vertex] = vec3(attrib.normals.data()+v);
			}
			return cloud->n_vertices() > 0;
        }
    }
}



namespace scan3d {

    namespace io {


        bool save_obj(const std::string &file_name, const PointCloud *cloud) {
			if (!cloud) {
				LOG(ERROR) << "null point cloud pointer";
				return false;
			}

			FILE *out = fopen(file_name.c_str(), "w");
			if (!out) {
				LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

			// comment
			fprintf(out, "# OBJ exported from Scan3D\n");
			//vertices
			const auto points = cloud->points();
			for (int i = 0; i < points.size(); i++ ) {
				const vec3 &p = points[i];
				fprintf(out, "v %.10f %.10f %.10f\n", p[0], p[1], p[2]);
			}

			//normals
			auto normals = cloud->get_vertex_property<vec3>("v:normal");
			if (normals) {
				for (PointCloud::VertexIterator vit = cloud->vertices_begin(); vit != cloud->vertices_end(); ++vit) {
					const vec3 &p = normals[*vit];
					fprintf(out, "vn %.10f %.10f %.10f\n", p[0], p[1], p[2]);
				}
			}
			fclose(out);
			return true;
        }


    } // namespace io

} // namespace scan3d

