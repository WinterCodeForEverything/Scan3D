#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_texturing.h>
#include <module/algo/surface_mesh_texturing_interface.h>
#include <MVS/Common.h>
#include <MVS/Scene.h>

#define CAMERA_FILEMVS	"/camera.mvs"

namespace scan3d {
	using namespace SEACAVE;

	SurfaceMeshTexturing::SurfaceMeshTexturing()
	{
		// initialize log and console
		OPEN_LOG();
		OPEN_LOGCONSOLE();
	}

	SurfaceMeshTexturing::~SurfaceMeshTexturing()
	{
		CLOSE_LOGCONSOLE();
		CLOSE_LOG();
	}

	bool SurfaceMeshTexturing::texturing( SurfaceMesh* mesh, 
		const std::string& strWorkingFolder, 
		const std::string& strRelativePose,
		const std::string& strSaveFile,
		const int nResolutionLevel,
		const int nMinResolution,
		const float fOutlierThreshold,
		const float fRatioDataSmoothness,
		const bool bGlobalSeamLeveling,
		const bool bLocalSeamLeveling,
		const int nTextureSizeMultiple,
		const int nRectPackingHeuristic,
		const int nColEmpty,
		const int nMaxThreads,
		const bool bImageType
		)
	{
		MVS::Scene scene(nMaxThreads);

		// camera
		SurfaceMeshTexturingInterface smti;
		smti.strImageType = bImageType ? ".jpg" : ".png";
		if (!smti.saveCameraFile( strWorkingFolder, strRelativePose ))
			return false;

		String strCameraFileName = strWorkingFolder + CAMERA_FILEMVS;
		if (!scene.Load(MAKE_PATH_SAFE(strCameraFileName)))
			return false;
		else 
			smti.deleteCamera();

		//input mesh
		if (!mesh->is_triangle_mesh())
			mesh->triangulate();
		
		//get vertices
		const std::vector<vec3>& points = mesh->points();
		std::vector<SEACAVE::Point3f> vertices(points.size());
		for (unsigned int i = 0; i < points.size(); i++)
		{
			vertices[i].x = points[i].x;
			vertices[i].y = points[i].y;
			vertices[i].z = points[i].z;
		}
		scene.mesh.vertices.CopyOf(&vertices[0], (uint32_t)vertices.size());
		
		//get vertex normals
		auto mesh_vertex_normals = mesh->get_vertex_property<vec3>("v:normal");
		if (mesh_vertex_normals)
		{
			std::vector<vec3> vNormal = mesh_vertex_normals.vector();
			std::vector<SEACAVE::Point3f> normals(vNormal.size());
			for (unsigned int i = 0; i < vNormal.size(); i++)
			{
				normals[i].x = vNormal[i].x;
				normals[i].y = vNormal[i].y;
				normals[i].z = vNormal[i].z;
			}
			scene.mesh.vertexNormals.CopyOf(&normals[0], (uint32_t)normals.size());
		}
		//get faces
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
			scene.mesh.faces.AddConstruct( id[0], id[1], id[2] );
		}
		// texturing
		if ( !scene.TextureMesh(nResolutionLevel, nMinResolution, fOutlierThreshold, fRatioDataSmoothness, bGlobalSeamLeveling, bLocalSeamLeveling, nTextureSizeMultiple, nRectPackingHeuristic, Pixel8U(nColEmpty)) )
			return false;

		// save textured model and image
		if (!scene.mesh.Save(strSaveFile))
			return false;

		//add texture coordinates to mesh
		auto prop_texcoords = mesh->halfedge_property<vec2>("h:texcoord");
		auto find_face_halfedge = [](SurfaceMesh *mesh, SurfaceMesh::Face face,
			SurfaceMesh::Vertex v) -> SurfaceMesh::Halfedge {
			for (auto h : mesh->halfedges(face)) {
				if (mesh->target(h) == v)
					return h;
			}
			LOG_FIRST_N(ERROR, 1) << "could not find a halfedge pointing to " << v << " in face " << face
				<< " (this is the first record)";
			return SurfaceMesh::Halfedge();
		};

		auto face_textures = scene.mesh.faceTexcoords;
		for (auto f : faces)
		{
			auto v = mesh->vertices(f);
			auto begin = find_face_halfedge(mesh, f, *v ), cur = begin;
			int id = 0;
			do
			{
				auto tex_coords = face_textures[f.idx()*3+id];
				prop_texcoords[cur] = vec2(tex_coords.x, tex_coords.y);
				++id;
				cur = mesh->next(cur);
			} while ( begin != cur );
		}
		
		return true;
	}
} // namespace scan3d