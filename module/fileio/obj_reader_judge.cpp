#include<module/fileio/obj_reader_judge.h>
#include <module/util/file_system.h>
#include <module/util/logging.h>
#include <3rd_party/tinyobjloader/tiny_obj_loader.h>

#include <module/core/point_cloud.h>
#include <module/core/surface_mesh.h>
#include <module/core/manifold_builder.h>
#include <module/util/file_system.h>
#include <module/util/logging.h>
namespace scan3d {

	namespace io {

		//bool ObjReaderWriter::load_obj2(const std::string &file_name, Model *model) {
		//	if (!model) {
		//		LOG(ERROR) << "null model pointer";
		//		return false;
		//	}
		//
		//	if (!file_system::is_file(file_name)) {
		//		LOG(ERROR) << "file does not exist: " << file_system::simple_name(file_name);
		//		return false;
		//	}
		//
		//	tinyobj::ObjReaderConfig config;
		//	config.triangulate = false;
		//	config.vertex_color = false;
		//	tinyobj::ObjReader reader;
		//	if (!reader.ParseFromFile(file_name, config)) {
		//		LOG(ERROR) << "failed parsing file: " + file_name + ". " << reader.Error() << reader.Warning();
		//		return false;
		//	}
		//
		//	// --------------------- collect the data ------------------------
		//	const tinyobj::attrib_t &attrib = reader.GetAttrib();
		//
		//	const std::vector<tinyobj::shape_t> &shapes = reader.GetShapes();
		//	if (shapes.empty()) {
		//		PointCloud * cloud = dynamic_cast<PointCloud *>(model);
		//		cloud->clear();
		//		// add vertices and attributes
		//		bool bNormal = false;
		//		PointCloud::VertexProperty<vec3> normal;
		//		if (attrib.normals.size())
		//		{
		//			bNormal = true;
		//			normal = cloud->vertex_property<vec3>("v:normal");
		//		}
		//		for (std::size_t v = 0; v < attrib.vertices.size(); v += 3) {
		//			auto vertex = cloud->add_vertex(vec3(attrib.vertices.data() + v));
		//			if (bNormal)
		//				normal[vertex] = vec3(attrib.normals.data() + v);
		//		}
		//		return cloud->n_vertices() > 0;
		//	}
		//
		//	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(model);
		//	const std::vector<tinyobj::material_t> &materials = reader.GetMaterials();
		//
		//	// for each texcoord
		//	std::vector<vec2> texcoords;
		//	for (std::size_t v = 0; v < attrib.texcoords.size(); v += 2) {
		//		texcoords.emplace_back(vec2(attrib.texcoords.data() + v));
		//	}
		//
		//	mesh->clear();
		//
		//	ManifoldBuilder builder(mesh);
		//	builder.begin_surface();
		//
		//	// add vertices
		//	for (std::size_t v = 0; v < attrib.vertices.size(); v += 3) {
		//		// Should I create vertices later, to get rid of isolated vertices?
		//		builder.add_vertex(vec3(attrib.vertices.data() + v));
		//	}
		//
		//	// create texture coordinate property if texture coordinates present
		//	if (!texcoords.empty())
		//		mesh->add_halfedge_property<vec2>("h:texcoord");
		//	auto prop_texcoords = mesh->get_halfedge_property<vec2>("h:texcoord");
		//
		//	auto find_face_halfedge = [](SurfaceMesh *mesh, SurfaceMesh::Face face,
		//		SurfaceMesh::Vertex v) -> SurfaceMesh::Halfedge {
		//		for (auto h : mesh->halfedges(face)) {
		//			if (mesh->target(h) == v)
		//				return h;
		//		}
		//		LOG_FIRST_N(ERROR, 1) << "could not find a halfedge pointing to " << v << " in face " << face
		//			<< " (this is the first record)";
		//		return SurfaceMesh::Halfedge();
		//	};
		//
		//	// invalid face will also be added, to ensure correct face indices
		//	std::vector<SurfaceMesh::Face> faces;
		//	// for each shape
		//	for (size_t i = 0; i < shapes.size(); i++) {
		//		std::size_t index_offset = 0;
		//		LOG_IF(ERROR, shapes[i].mesh.num_face_vertices.size() != shapes[i].mesh.material_ids.size());
		//		LOG_IF(ERROR, shapes[i].mesh.num_face_vertices.size() != shapes[i].mesh.smoothing_group_ids.size());
		//
		//		// For each face
		//		for (std::size_t face_idx = 0; face_idx < shapes[i].mesh.num_face_vertices.size(); ++face_idx) {
		//			std::size_t face_size = shapes[i].mesh.num_face_vertices[face_idx];
		//
		//			// For each vertex in the face
		//			std::vector<SurfaceMesh::Vertex> vertices;
		//			std::vector<int> texcoord_ids;
		//			for (std::size_t v = 0; v < face_size; v++) {
		//				const tinyobj::index_t &face = shapes[i].mesh.indices[index_offset + v];
		//				vertices.emplace_back(face.vertex_index);
		//				if (prop_texcoords)
		//					texcoord_ids.emplace_back(face.texcoord_index);
		//			}
		//
		//			SurfaceMesh::Face face = builder.add_face(vertices);
		//			// invalid face will also be added, to ensure correct face indices
		//			faces.push_back(face);
		//			if (prop_texcoords && face.is_valid()) {
		//				auto begin = find_face_halfedge(mesh, face, builder.face_vertices()[0]);
		//				auto cur = begin;
		//				int idx = 0;
		//				do {
		//					prop_texcoords[cur] = texcoords[texcoord_ids[idx++]];
		//					cur = mesh->next(cur);
		//				} while (cur != begin);
		//			}
		//
		//			//int smoothing_group_id = shapes[i].mesh.smoothing_group_ids[f];
		//			index_offset += face_size;
		//		}
		//	}
		//
		//	builder.end_surface();
		//
		//	// now the material
		//	if (!materials.empty()) {
		//		auto face_color = mesh->add_face_property<vec3>("f:color");
		//		int face_idx = 0;
		//		for (std::size_t i = 0; i < shapes.size(); i++) {
		//			for (std::size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
		//				auto face = faces[face_idx];
		//				if (face.is_valid()) {
		//					// the material id of this face
		//					const int material_id = shapes[i].mesh.material_ids[f];
		//					const tinyobj::material_t &mat = materials[material_id];
		//					// ambient = vec3(mat.ambient);
		//					// specular = vec3(mat.specular);
		//					face_color[face] = vec3(mat.diffuse);
		//				}
		//				++face_idx;
		//			}
		//		}
		//
		//		for (const auto &mat : materials) {
		//			LOG_IF(WARNING, !mat.ambient_texname.empty()) << "ambient texture ignored: " << mat.ambient_texname;
		//			LOG_IF(WARNING, !mat.diffuse_texname.empty()) << "diffuse texture ignored: " << mat.diffuse_texname;
		//			LOG_IF(WARNING, !mat.specular_texname.empty())
		//				<< "specular texture ignored: " << mat.specular_texname;
		//		}
		//	}
		//
		//	return mesh->n_faces() > 0;
		//}


		bool ObjReaderJudge::obj_file_is_valid()
		{
			if (!file_system::is_file(_file_name)) {
				LOG(ERROR) << "file does not exist: " << file_system::simple_name(_file_name);
				return false;
			}

			tinyobj::ObjReaderConfig config;
			config.vertex_color = false;
			tinyobj::ObjReader reader;
			if (!reader.ParseFromFile(_file_name, config)) {
				LOG(ERROR) << "failed parsing file: " + _file_name + ". " << reader.Error() << reader.Warning();
				return false;
			}
			return true;
		}
		bool ObjReaderJudge::obj_is_mesh()
		{
			tinyobj::ObjReaderConfig config;
			config.vertex_color = false;
			tinyobj::ObjReader reader;
			reader.ParseFromFile(_file_name, config);
			const std::vector<tinyobj::shape_t> &shapes = reader.GetShapes();
			if (shapes.empty()) {
			    return false;
			}
			return true;
		}
	} 
}

