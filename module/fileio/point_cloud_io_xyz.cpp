#include <module/fileio/point_cloud_io.h>

#include <fstream>

#include <module/core/point_cloud.h>
#include <module/util/line_stream.h>
#include <module/util/logging.h>
#include <module/util/progress.h>


namespace scan3d {

	namespace io {

		bool load_xyz(const std::string& file_name, PointCloud* cloud) {
			std::ifstream input(file_name.c_str());
			if (input.fail()) {
                LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

			cloud->clear();
			io::LineInputStream in(input);

			// get length of file
            input.seekg(0, input.end);
            std::streamoff length = input.tellg();
            input.seekg(0, input.beg);
            ProgressLogger progress(length);

			vec3 p, n;
			bool bNormal = false;
			PointCloud::VertexProperty<vec3> normals;
			while (!input.eof()) {
				in.get_line();
				if (in.current_line()[0] != '#') {
					in >> p;
					if (!in.fail()) {
						auto vertex = cloud->add_vertex(p);
						in >> n;
						if (!in.fail()) {
							if (!bNormal)
							{
								normals = cloud->vertex_property<vec3>("v:normal");
								bNormal = true;
							}
							normals[vertex] = n;
						}
                        std::streamoff pos = input.tellg();
                        progress.notify(pos);
					}

				}
			}

			return true;
		}


		bool save_xyz(const std::string& file_name, const PointCloud* cloud) {
			std::ofstream output(file_name.c_str());
			if (output.fail()) {
                LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}
			output.precision(16);

			PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");

            ProgressLogger progress(cloud->n_vertices());
            for (auto v : cloud->vertices()) {
                output << points[v] << std::endl;
                progress.next();
            }

			return true;
		}


		bool load_bxyz(const std::string& file_name, PointCloud* cloud) {
			std::ifstream input(file_name.c_str(), std::fstream::binary);
			if (input.fail()) {
                LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

			std::size_t element_per_point = 3;
			std::size_t element_size = sizeof(float) * element_per_point;
			// check size of types
			if (sizeof(vec3) != sizeof(float) * 3) {
				LOG(ERROR) << "point coordinates must be \'float\'";
				return false;
			}

			// get length of file
			input.seekg(0, input.end);
			std::streamoff length = input.tellg();
			input.seekg(0, input.beg);

			// num of points in the file
			std::size_t num = length / element_size;
			if (num <= 0)
				return false;

			cloud->resize(static_cast<unsigned int>(num));

			PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");

			input.read((char*)points.data(), num * element_size);	// read the entire blocks
			if (input.fail()) {
				LOG(ERROR)
					<< input.gcount() << " bytes of the block could not be read";
				return false;
			}
			return true;
		}


		bool save_bxyz(const std::string& file_name, const PointCloud* cloud) {
			// open file
			std::ofstream output(file_name.c_str(), std::fstream::binary);
			if (output.fail()) {
				LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}

			// check size of types
			if (sizeof(vec3) != sizeof(float) * 3) {
				LOG(ERROR) << "point coordinates must be \'float\'";
				return false;
			}

			PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");

			output.write((const char*)points.data(), sizeof(vec3) * cloud->n_vertices());
			return true;
		}

	} // namespace io

} // namespace scan3d
