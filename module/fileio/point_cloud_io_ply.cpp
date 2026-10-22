#include <module/fileio/point_cloud_io.h>

#include <unordered_map>

#include <module/fileio/ply_reader_writer.h>
#include <module/core/point_cloud.h>
#include <module/core/types.h>
#include <module/util/logging.h>


namespace scan3d {

	namespace io {

		namespace details {

			template <typename T, typename PropertyT>
			inline void add_properties(PointCloud* cloud, const std::vector<PropertyT>& properties)
			{
				for (const auto& p : properties) {
                    std::string name = p.name;
					if (name.find("v:") == std::string::npos)
						name = "v:" + name;
					auto prop = cloud->vertex_property<T>(name);
					prop.vector() = p;
				}
			}

		} // namespace details

		bool load_ply(const std::string& file_name, PointCloud* cloud) {
			std::vector<Element> elements;
			PlyReader reader;
			if (!reader.read(file_name, elements))
				return false;

            for (std::size_t i = 0; i < elements.size(); ++i) {
                const Element& e = elements[i];
                if (e.name == "vertex") {
                    cloud->resize(static_cast<unsigned int>(e.num_instances));
                    break;
                }
            }

			for (std::size_t i = 0; i < elements.size(); ++i) {
				const Element& e = elements[i];
                if (e.name == "vertex") {
                    details::add_properties<vec3>(cloud, e.vec3_properties);
                    details::add_properties<vec2>(cloud, e.vec2_properties);
                    details::add_properties<float>(cloud, e.float_properties);
                    details::add_properties<int>(cloud, e.int_properties);
                    details::add_properties< std::vector<int> >(cloud, e.int_list_properties);
                    details::add_properties< std::vector<float> >(cloud, e.float_list_properties);
                }
                else {
                    const std::string name = "element-" + e.name;
                    auto prop = cloud->add_model_property<Element>(name, Element(""));
                    prop.vector().push_back(e);
                    LOG(WARNING) << "unknown element '" << e.name
                                 << "' with the following properties has been stored as model property '" << name << "'"
                                 << e.property_statistics();
                }
			}

			return true;
		}


		namespace details {

			template <typename T>
			inline void collect_properties(const PointCloud* cloud, std::vector< GenericProperty<T> >& properties) {
				const auto& all_properties = cloud->vertex_properties();
				for (auto name : all_properties) {
					auto prop = cloud->get_vertex_property<T>(name);
					if (prop) {
						if (name.substr(0, 2) == "v:")
							name = name.substr(2, name.length() - 1);
						properties.emplace_back(GenericProperty<T>(name, prop.vector()));
					}
				}
			}

		} // namespace details

		bool save_ply(const std::string& file_name, const PointCloud* cloud, bool binary/* = true*/) {
			if (!cloud || cloud->n_vertices() == 0) {
				LOG(ERROR) << "empty point cloud data";
				return false;
			}

			std::size_t num = cloud->n_vertices();
			Element e("vertex", num);

			details::collect_properties(cloud, e.vec3_properties);
            details::collect_properties(cloud, e.vec2_properties);
			details::collect_properties(cloud, e.float_properties);
			details::collect_properties(cloud, e.int_properties);
			details::collect_properties(cloud, e.int_list_properties);
			details::collect_properties(cloud, e.float_list_properties);

			std::vector<Element> elements;
            elements.emplace_back(e);

            binary = binary && (file_name.find("ascii") == std::string::npos);
            LOG_IF(WARNING, !binary) << "you're writing an ASCII ply file. Use binary format for better performance";

            PlyWriter writer;
            return writer.write(file_name, elements, "", binary);
		}


	} // namespace io

} // namespace scan3d
