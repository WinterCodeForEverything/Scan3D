#include <module/fileio/point_cloud_io.h>

#include <fstream>

#include <module/core/point_cloud.h>


namespace scan3d {


	namespace io {


		// three blocks storing points, colors (optional), and normals (optional)
		bool load_bin(const std::string& file_name, PointCloud* cloud) {
			std::ifstream input(file_name.c_str(), std::fstream::binary);
			if (input.fail()) {
                LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

            int num = 0;
            input.read((char*)(&num), sizeof(int));
            if (num <= 0) {
				LOG(ERROR) << "no point exists in file: " << file_name;
				return false;
			}
            cloud->resize(num);

			// read the points block
			PointCloud::VertexProperty<vec3> points = cloud->vertex_property<vec3>("v:point");
            input.read((char*)points.data(), num * sizeof(vec3));

            // read the colors block if exists
            input.read((char*)(&num), sizeof(int));
            if (num > 0) {
				PointCloud::VertexProperty<vec3> colors = cloud->vertex_property<vec3>("v:color");
                input.read((char*)colors.data(), num * sizeof(vec3));
			}

            // read the normals block if exists
            input.read((char*)(&num), sizeof(int));
            if (num > 0) {
				PointCloud::VertexProperty<vec3> normals = cloud->vertex_property<vec3>("v:normal");
                input.read((char*)normals.data(), num * sizeof(vec3));
                // check if the normals are normalized
                const float len = length(normals[PointCloud::Vertex(0)]);
                LOG_IF(WARNING, std::abs(1.0 - len) > epsilon<float>())
                                << "normals not normalized (length of the first normal vector is " << len << ")";
			}

			return cloud->n_vertices() > 0;
		}


		bool save_bin(const std::string& file_name, const PointCloud* cloud) {
			// open file
			std::ofstream output(file_name.c_str(), std::fstream::binary);
			if (output.fail()) {
                LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

            int num = cloud->n_vertices();

			// write the points block
			auto points = cloud->get_vertex_property<vec3>("v:point");
            output.write((char*)&num, sizeof(int));
			output.write((char*)points.data(), num * sizeof(vec3));

			auto colors = cloud->get_vertex_property<vec3>("v:color");
			if (colors) {
                output.write((char*)&num, sizeof(int));
				output.write((char*)colors.data(), num * sizeof(vec3));
			}
            else {
                int num_colors = 0;
                output.write((char*)&num_colors, sizeof(int));
            }

			auto normals = cloud->get_vertex_property<vec3>("v:normal");
			if (normals) {
                output.write((char*)&num, sizeof(int));
				output.write((char*)normals.data(), num * sizeof(vec3));
			}
            else {
                int num_normals = 0;
                output.write((char*)&num_normals, sizeof(int));
            }

			return true;
		}

	} // namespace io

} // namespace scan3d
