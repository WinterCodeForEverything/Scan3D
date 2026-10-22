#ifndef SCAN3D_ALGO_SURFACE_MESH_TRIANGULATION_H
#define SCAN3D_ALGO_SURFACE_MESH_TRIANGULATION_H


#include <module/core/surface_mesh.h>

#include <vector>
#include <float.h>


namespace scan3d {

    /**
     * \brief Triangulate a polygonal mesh into a pure triangle mesh.
     * \details Tringulate n-gons into n-2 triangles. Find the triangulation that minimizes the sum of squared triangle
     * areas. See the following paper for more details:
     *  - Peter Liepa. Filling holes in meshes. SGP, 2003.
     */
    class SurfaceMeshTriangulation {
    public:

        //! triangulation objective: find the triangulation that minimizes the
        //! sum of squared triangle areas, or the one that maximizes the minimum
        //! angle.
        enum Objective {
            MIN_AREA, MAX_ANGLE
        } objective_;

        SurfaceMeshTriangulation(SurfaceMesh *mesh);

        //! triangulate all faces
        void triangulate(Objective obj = MIN_AREA);

        //! triangulate a particular face f
        void triangulate(SurfaceMesh::Face f, Objective obj = MIN_AREA);

    private:

        // compute the weight of the triangle (i,j,k).
        float compute_weight(int i, int j, int k) const;

        // does edge (a,b) exist?
        bool is_edge(SurfaceMesh::Vertex a, SurfaceMesh::Vertex b) const;

        // does edge (a,b) exist and is non-boundary?
        bool is_interior_edge(SurfaceMesh::Vertex a, SurfaceMesh::Vertex b) const;

        // add edges from vertex i to j
        bool insert_edge(int i, int j);


    private:

        // mesh and properties
        SurfaceMesh *mesh_;
        SurfaceMesh::VertexProperty <vec3> points_;
        std::vector<SurfaceMesh::Halfedge> halfedges_;
        std::vector<SurfaceMesh::Vertex> vertices_;

        // data for computing optimal triangulation
        std::vector<std::vector<float> > weight_;
        std::vector<std::vector<int> > index_;
    };

} // namespace scan3d


#endif  // SCAN3D_ALGO_SURFACE_MESH_TRIANGULATION_H