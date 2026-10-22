#ifndef SCAN3D_ALGO_SURFACE_MESH_GEOMETRY_H
#define SCAN3D_ALGO_SURFACE_MESH_GEOMETRY_H


#include <module/core/types.h>
#include <module/core//surface_mesh.h>


namespace scan3d {

    namespace geom {

        /** compute area of triangle f    */
        float triangle_area(const SurfaceMesh *mesh, SurfaceMesh::Face f);

        /** surface area of the mesh (assumes triangular faces)    */
        float surface_area(const SurfaceMesh *mesh);

        /** barycenter/centroid of a face    */
        vec3 centroid(const SurfaceMesh *mesh, SurfaceMesh::Face f);

        /** barycenter/centroid of mesh, computed as area-weighted mean of vertices.    */
        /** assumes triangular faces.    */
        vec3 centroid(const SurfaceMesh *mesh);

        /** compute the cotangent weight for edge e    */
        double cotan_weight(const SurfaceMesh *mesh, SurfaceMesh::Edge e);

        /** compute (mixed) Voronoi area of vertex v    */
        double voronoi_area(const SurfaceMesh *mesh, SurfaceMesh::Vertex v);

        /** compute barycentric Voronoi area of vertex v    */
        double voronoi_area_barycentric(const SurfaceMesh *mesh, SurfaceMesh::Vertex v);

        /** compute Laplace vector for vertex v (normalized by Voronoi area)    */
        vec3 laplace(const SurfaceMesh *mesh, SurfaceMesh::Vertex v);

        /** compute the sum of angles around vertex v (used for Gaussian curvature)    */
        float angle_sum(const SurfaceMesh *mesh, SurfaceMesh::Vertex v);

        /** discrete curvature information for a vertex. used for vertex_curvature()    */
        struct VertexCurvature {
            VertexCurvature() : mean(0.0), gauss(0.0), max(0.0), min(0.0) {}

            float mean;
            float gauss;
            float max;
            float min;
        };

        /** compute min, max, mean, and Gaussian curvature for vertex v. this will not    */
        /** give reliable values for boundary vertices.    */
        VertexCurvature vertex_curvature(const SurfaceMesh *mesh, SurfaceMesh::Vertex v);

    }

} // namespace scan3d


#endif  // SCAN3D_ALGO_SURFACE_MESH_GEOMETRY_H