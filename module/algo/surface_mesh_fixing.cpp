#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_fixing.h>
#include <3rd_party/MeshFix/include/TMesh/tmesh.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <map>

namespace scan3d {

	using namespace T_MESH;
	inline Basic_TMesh* convertSurfaceMeshtoTMesh( SurfaceMesh *mesh )
	{
		Basic_TMesh* tin = new Basic_TMesh;
		for (auto p : mesh->points())
			tin->V.appendTail(tin->newVertex(p.x, p.y, p.z));
		int nv = tin->V.numels();
		ExtVertex **var = (ExtVertex **)malloc(sizeof(ExtVertex *)*nv);
		int i = 0;
		for ( Node *n = tin->V.head(); n != NULL; n = n->next() )
		{
			var[i++] = new ExtVertex((Vertex *)n->data);
		}

		//
		const auto faces = mesh->faces();
		for (auto f : faces)
		{
			auto v = mesh->vertices(f), vend = v;
			int i = 0, id[3];
			do
			{
				assert(i < 3);
				id[i++] = (*v).idx();
			} while (++v != vend);
			if (!tin->CreateIndexedTriangle(var, id[0], id[1], id[2]) )
				return NULL;
		}
		return tin;
	}

	inline SurfaceMesh* convertTMeshtoSurfaceMesh( Basic_TMesh* tmesh )
	{
		SurfaceMesh* mesh = new SurfaceMesh;
		std::vector<SurfaceMesh::Vertex> all_vertices;
		std::map<Vertex*, int > mVertices;
		int i = 0;
		for (Node *n = tmesh->V.head(); n != NULL; n = n->next())
		{
			Vertex *vertex = (Vertex *)n->data;
			mVertices[vertex] = i++;
			SurfaceMesh::Vertex vv = mesh->add_vertex(vec3(vertex->x, vertex->y, vertex->z));
			all_vertices.push_back(vv);
		}
		for (Node *n = tmesh->T.head(); n != NULL; n = n->next())
		{
			Triangle *triangle = (Triangle*)n->data;
			std::vector<SurfaceMesh::Vertex> face_vts;
			face_vts.push_back(all_vertices[mVertices[triangle->v1()]]);
			face_vts.push_back(all_vertices[mVertices[triangle->v2()]]);
			face_vts.push_back(all_vertices[mVertices[triangle->v3()]]);
			mesh->add_face(face_vts);
		}
		return mesh;
	}
	SurfaceMesh* SurfaceMeshFixing::removeSelfInteraction(SurfaceMesh *mesh)
	{
		if (!mesh)
		{
			LOG(ERROR) << "Can't get proper mesh!";
			return NULL;
		}
		if (!mesh->is_triangle_mesh())
			mesh->triangulate();
		Basic_TMesh* tin = convertSurfaceMeshtoTMesh(mesh);
		if (!tin)
		{
			LOG(ERROR) << "Can't convert scan3d SurfaceMesh to TMesh!";
			return NULL;
		}

		// Keep only the largest component (i.e. with most triangles)
		int sc = tin->removeSmallestComponents();
		if (sc)
			LOG(INFO) << "Removed " << sc << " small components";
		
		// Fill holes
		if (tin->boundaries())
		{
			LOG(INFO) << "Patching holes\n";
			tin->fillSmallBoundaries(0, true);
		}

		// Run geometry correction
		if (!tin->boundaries()) 
			LOG(INFO) << "Fixing degeneracies and intersections...";
		if (tin->boundaries() || !tin->meshclean())
			LOG(ERROR) << "MeshFix could not fix self-interaction!";

		LOG(INFO) << mesh->name();
		//tin->save(mesh->name().c_str());
		SurfaceMesh* newMesh = convertTMeshtoSurfaceMesh(tin);
		return newMesh;

	}

} // namespace scan3d