#include <module/fileio/surface_mesh_io.h>

#include <module/core/surface_mesh.h>
#include <module/core/manifold_builder.h>
#include <Cpp/lib3mf_implicit.hpp>

namespace scan3d {

	namespace io {

	using namespace Lib3MF;

		bool load_3mf( const std::string& file_name, SurfaceMesh* mesh )
		{
			auto wrapper = CWrapper::loadLibrary();
			PModel model = wrapper->CreateModel();
			PReader reader = model->QueryReader("3mf");
			reader->SetStrictModeActive(false);
			reader->ReadFromFile(file_name);
			for (Lib3MF_uint32 iWarning = 0; iWarning < reader->GetWarningCount(); iWarning++) {
				Lib3MF_uint32 nErrorCode;
				std::string sWarningMessage = reader->GetWarning(iWarning, nErrorCode);
				LOG(ERROR) << "Encountered warning #" << nErrorCode << " : " << sWarningMessage << std::endl;
			}

			// get surface information
			auto objectIterator = model->GetObjects();
			std::vector<sPosition> vMeshVertice;
			std::vector<sTriangle> vMeshTrianglesInd;
			//std::vector<sTriangleProperties> vMeshTriangleProperties;
			while (objectIterator->MoveNext()) {
				PObject object = objectIterator->GetCurrentObject();
				if (object->IsMeshObject()) {
					PMeshObject meshObject = model->GetMeshObjectByID(object->GetResourceID());
					meshObject->GetVertices(vMeshVertice);
					meshObject->GetTriangleIndices(vMeshTrianglesInd);
					//meshObject->GetAllTriangleProperties(vMeshTriangleProperties);
				}
			}

			// get texture
			//auto textureIterator = model->GetTexture2DGroups();
			//std::vector<vec2> texcoords;
			//while (textureIterator->MoveNext()) {
			//	auto texture2Dgroup = textureIterator->GetCurrentTexture2DGroup();
			//	for (Lib3MF_uint32 i = 0; i < texture2Dgroup->GetCount(); i++)
			//		for (Lib3MF_uint32 j = 0; j < 3; j++)
			//		{
			//			sTex2Coord tex = texture2Dgroup->GetTex2Coord(vMeshProperties[i].m_PropertyIDs[j]);
			//			texcoords.emplace_back(vec2(tex.m_U, tex.m_V));
			//		}
			//	//break;
			//}

			//std::vector<vec2> texcoords;


			// clear the mesh in case of existing data
			mesh->clear();

			ManifoldBuilder builder(mesh);

			//auto find_face_halfedge = [](SurfaceMesh *mesh, SurfaceMesh::Face face,
			//	SurfaceMesh::Vertex v) -> SurfaceMesh::Halfedge {
			//	for (auto h : mesh->halfedges(face)) {
			//		if (mesh->target(h) == v)
			//			return h;
			//	}
			//	LOG_FIRST_N(ERROR, 1) << "could not find a halfedge pointing to " << v << " in face " << face
			//		<< " (this is the first record)";
			//	return SurfaceMesh::Halfedge();
			//};

			//// create texture coordinate property if texture coordinates present
			//if (!vMeshTriangleProperties.empty())
			//	mesh->add_halfedge_property<vec2>("h:texcoord");
			//auto prop_texcoords = mesh->get_halfedge_property<vec2>("h:texcoord");

			builder.begin_surface();

			//add vertices
			std::vector<SurfaceMesh::Vertex> vTmpVertices(vMeshVertice.size());
			for ( int i = 0; i < vMeshVertice.size(); i++ )
			{
				vTmpVertices[i] = builder.add_vertex( vec3( vMeshVertice[i].m_Coordinates[0], vMeshVertice[i].m_Coordinates[1], vMeshVertice[i].m_Coordinates[2] ) );
			}
			// invalid face will also be added, to ensure correct face indices
			for (size_t i = 0; i < vMeshTrianglesInd.size(); i++) {
				auto aTriangleInd = vMeshTrianglesInd[i];
				auto face = builder.add_triangle( vTmpVertices[aTriangleInd.m_Indices[0]], vTmpVertices[aTriangleInd.m_Indices[1]], vTmpVertices[aTriangleInd.m_Indices[2]]);

				////add texture
				//if (prop_texcoords && face.is_valid())
				//{
				//	sTriangleProperties currentProperty = vMeshTriangleProperties[i];
				//	auto currentTexture2DGroup = model->GetTexture2DGroupByID(currentProperty.m_ResourceID);
				//	auto begin = find_face_halfedge(mesh, face, builder.face_vertices()[0]);
				//	auto cur = begin;
				//	int idx = 0;
				//	do {
				//		sTex2Coord uvcoord = currentTexture2DGroup->GetTex2Coord(currentProperty.m_PropertyIDs[idx++]);
				//		prop_texcoords[cur] = vec2(uvcoord.m_U, uvcoord.m_V);
				//		cur = mesh->next(cur);
				//	} while (cur != begin);
				//}
			}
			builder.end_surface();
			
			return mesh->n_faces() > 0;
		}

		bool save_3mf( const std::string& file_name, const SurfaceMesh* mesh )
		{
			if (!mesh) {
				LOG(ERROR) << "null mesh pointer";
				return false;
			}

			FILE *out = fopen(file_name.c_str(), "w");
			if (!out) {
				LOG(ERROR) << "could not open file: " << file_name;
				return false;
			}
			fclose(out);

			//
			SurfaceMesh save_mesh(*mesh);
			if (!save_mesh.is_triangle_mesh())
				save_mesh.triangulate();

			auto wrapper = CWrapper::loadLibrary();
			PModel model = wrapper->CreateModel();
			PWriter writer = model->QueryWriter("3mf");
			
			//save points
			sPosition sP;
			std::vector<sPosition> vMeshVertice;
			SurfaceMesh::VertexProperty<vec3> points = save_mesh.get_vertex_property<vec3>("v:point");
			for (SurfaceMesh::VertexIterator vit = save_mesh.vertices_begin(); vit != save_mesh.vertices_end(); ++vit) {
				const vec3 &p = points[*vit];
				sP.m_Coordinates[0] = p[0];
				sP.m_Coordinates[1] = p[1];
				sP.m_Coordinates[2] = p[2];
				vMeshVertice.push_back( sP );
			}

			//save faces 
			sTriangle sT;
			std::vector<sTriangle> vMeshTrianglesInd;
			for ( SurfaceMesh::FaceIterator fit = save_mesh.faces_begin(); fit != save_mesh.faces_end(); ++fit ) {
				SurfaceMesh::VertexAroundFaceCirculator fvit = save_mesh.vertices(*fit), fvend = fvit;
				//SurfaceMesh::HalfedgeAroundFaceCirculator fhit = save_mesh.halfedges(*fit);
				int idx = 0;
				do {
					sT.m_Indices[idx++] = (*fvit).idx();
				} while ( ++fvit != fvend );
				vMeshTrianglesInd.push_back( sT );
			}

			auto meshObject = model->AddMeshObject();
			meshObject->SetGeometry(vMeshVertice, vMeshTrianglesInd);
			writer->WriteToFile(file_name);
			return true;
		}
	} // namespace io

} // namespace easy3
