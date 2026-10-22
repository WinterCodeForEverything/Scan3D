#include<algorithm>
#include <module/algo/point_cloud_delaunay_reconstruction.h>

#include <module/core/point_cloud.h>
#include <module/core/surface_mesh.h>
#include <point2mesh/mesh/mesh.h>
#include <point2mesh/mesh/mesh_io.h>
#include <point2mesh/points/co3ne.h>
#include <point2mesh/mesh/mesh_geometry.h>
#include <point2mesh/mesh/mesh_repair.h>
#include <point2mesh/basic/common.h>
#include <point2mesh/basic/logger.h>
#include <point2mesh/basic/command_line.h>
#include <point2mesh/basic/command_line_args.h>
#include <point2mesh/basic/progress.h>
#include <point2mesh/basic/process.h>
#include <point2mesh/basic/file_system.h>
#include <point2mesh/mesh/mesh_remesh.h>
#include <point2mesh/mesh/mesh_preprocessing.h>
#include <point2mesh/delaunay/LFS.h>


namespace scan3d {

	inline void remove_small_components(GEO::Mesh& M, double min_comp_area) {
		if (min_comp_area == 0.0) { return; }
		double nb_f_removed = M.facets.nb();
		GEO::remove_small_connected_components(M, min_comp_area);
		nb_f_removed -= M.facets.nb();
		if (nb_f_removed != 0) {
			double radius = bbox_diagonal(M);
			double epsilon = GEO::CmdLine::get_arg_percent("pre:epsilon", radius);
			mesh_repair(M, GEO::MESH_REPAIR_DEFAULT, epsilon);
		}
	}

/*
将点云的点坐标输入point2mesh的网格顶点坐标中
*/
	inline void set_mesh_point(
		GEO::Mesh& M, unsigned int v, const double* coords, unsigned int dim
	) {
		geo_debug_assert(M.vertices.dimension() >= dim);
		if (M.vertices.single_precision()) {
			float* p = M.vertices.single_precision_point_ptr(v);
			for (int c = 0; c < dim; ++c) {
				p[c] = float(coords[c]);
			}
		}
		else {
			double* p = M.vertices.point_ptr(v);
			for (int c = 0; c < dim; ++c) {
				p[c] = coords[c];
			}
		}
	}
	inline void set_repairing_para( double area, int edges, bool brepair )
	{
		GEO::CmdLine::set_arg("co3ne:repair", brepair);
		GEO::CmdLine::set_arg_percent("co3ne:max_hole_area", area );
		GEO::CmdLine::set_arg("co3ne:max_hole_edges", edges );
	}


	DelnaunayReconstruction::DelnaunayReconstruction()
	{
		//initialize interfaces
		GEO::FileSystem::initialize();
		GEO::Logger::initialize();
		GEO::Process::initialize(0);
		GEO::Progress::initialize();
		GEO::CmdLine::initialize();
		GEO::Delaunay::initialize();
		GEO::mesh_io_initialize();
		//import
		GEO::CmdLine::import_arg_group("standard");
		GEO::CmdLine::import_arg_group("pre");
		GEO::CmdLine::import_arg_group("remesh");
		GEO::CmdLine::import_arg_group("algo");
		GEO::CmdLine::import_arg_group("post");
		GEO::CmdLine::import_arg_group("opt");
		GEO::CmdLine::import_arg_group("co3ne");
		GEO::CmdLine::import_arg_group("tet");
		GEO::CmdLine::import_arg_group("poly");
	}

	DelnaunayReconstruction::~DelnaunayReconstruction()
	{
		//release interfaces and exit clean
		GEO::Progress::terminate();
		GEO::Process::terminate();
		GEO::CmdLine::terminate();
		GEO::Logger::terminate();
		GEO::FileSystem::terminate();
		GEO::Environment::terminate();
	}
/*
重建网格
*/
	SurfaceMesh *DelnaunayReconstruction::reconstruct(const PointCloud *cloud,
		const float radius,
		const int nb_iterations,
		const int nb_neighbors,
		const int nb_points,
		const int nb_Lloyd_iter,
		const int nb_Newton_iter,
		const int Newton_m,
		const bool bRemeshing,
		const bool bWaterTight
	)
	{
		//transform data type
		SurfaceMesh* result = new SurfaceMesh;
		GEO::Mesh M_in;
		//GEO::Mesh M_out;
		M_in.clear();
		M_in.vertices.set_double_precision();
		M_in.vertices.create_vertices(cloud->points().size());
		double xyz[3];
		const std::vector<vec3> &points = cloud->points();
		for (unsigned int i = 0; i < points.size(); i++ )
		{
			xyz[0] = points[i].x; xyz[1] = points[i].y; xyz[2] = points[i].z;
			set_mesh_point(M_in, i, xyz, 3);
		}
		

		// Remove all facets
		M_in.facets.clear();

		double bbox_diag = GEO::bbox_diagonal(M_in);
		GEO::mesh_repair(M_in, GEO::MESH_REPAIR_COLOCATE, 1e-6 * bbox_diag);

		if (bWaterTight)
		{
			set_repairing_para(5.0, 500, true);
			
		}
		else{
			set_repairing_para(0.01, 30, true);
		}

		
			

		double re_radius = 0.01*radius*bbox_diag;
		//
		GEO::Co3Ne_smooth_and_reconstruct(M_in, nb_neighbors, nb_iterations, re_radius );

		//remove_small_components( M_in, 0.1 );
		//remesh add user input as parameter(here 1000)
		if (bRemeshing) {
			GEO::remesh_smooth(M_in, M_in, nb_points, 0, nb_Lloyd_iter, nb_Newton_iter, Newton_m);
		}
			
		//if (M_out.vertices.nb() == 0)
		//	M_out = M_in;
		//transform data type back
		std::vector<SurfaceMesh::Vertex> all_vertices;
		for (unsigned int i = 0; i < M_in.vertices.nb(); i++ )
		{
			const GEO::vec3& vertex = M_in.vertices.point(i);
			SurfaceMesh::Vertex vv = result->add_vertex( vec3(vertex.x, vertex.y, vertex.z ));
			all_vertices.push_back(vv);
		}
		
		
		for (unsigned int i = 0; i < M_in.facets.nb(); i++)
		{
			std::vector<SurfaceMesh::Vertex> face_vts;

			for (unsigned int j = M_in.facets.corners_begin(i); j < M_in.facets.corners_end(i); ++j)
				face_vts.push_back( all_vertices[M_in.facet_corners.vertex(j)] );
			std::reverse(face_vts.begin(), face_vts.end());
			result->add_face(face_vts);

		}
		return result;
	}
};