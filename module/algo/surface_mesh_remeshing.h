#ifndef SCAN3D_ALGO_SURFACE_MESH_REMESHING_H
#define SCAN3D_ALGO_SURFACE_MESH_REMESHING_H

#include <module/core/surface_mesh.h>

namespace scan3d {

    class TriangleMeshKdTree;

    /**
     * A class for uniform and adaptive surface remeshing.
     * The algorithm implemented here performs incremental remeshing based on edge collapse, split, flip, and tangential
     * relaxation. See the following papers for more details:
     *  - Mario Botsch and Leif Kobbelt. A remeshing approach to multiresolution modeling. SGP, 2004.
     *  - Marion Dunyach et al. Adaptive remeshing for real-time mesh deformation. EG (Short Papers) 2013.
     */

    class SurfaceMeshRemeshing {
    public:
        SurfaceMeshRemeshing(SurfaceMesh *mesh);

        ~SurfaceMeshRemeshing();

        //! uniform remeshing with target edge length
        void uniform_remeshing(float edge_length, unsigned int iterations = 10,
                               bool use_projection = true);

        //! adaptive remeshing with min/max edge length and approximation error
        void adaptive_remeshing(float min_edge_length, float max_edge_length,
                                float approx_error, unsigned int iterations = 10,
                                bool use_projection = true);

    private:
        void preprocessing();
        void postprocessing();

        void split_long_edges();
        void collapse_short_edges();
        void flip_edges();
        void tangential_smoothing(unsigned int iterations);

        void remove_caps();

        void project_to_reference(SurfaceMesh::Vertex v);

        bool is_too_long(SurfaceMesh::Vertex v0, SurfaceMesh::Vertex v1) const {
            return distance(points_[v0], points_[v1]) >
                   4.0 / 3.0 * std::min(vsizing_[v0], vsizing_[v1]);
        }

        bool is_too_short(SurfaceMesh::Vertex v0, SurfaceMesh::Vertex v1) const {
            return distance(points_[v0], points_[v1]) <
                   4.0 / 5.0 * std::min(vsizing_[v0], vsizing_[v1]);
        }

    private:
        SurfaceMesh *mesh_;
        SurfaceMesh *refmesh_;

        bool use_projection_;
        TriangleMeshKdTree *kd_tree_;

        bool uniform_;
        float target_edge_length_;
        float min_edge_length_;
        float max_edge_length_;
        float approx_error_;

        SurfaceMesh::VertexProperty <vec3> points_;
        SurfaceMesh::VertexProperty <vec3> vnormal_;
        SurfaceMesh::VertexProperty<bool> vfeature_;
        SurfaceMesh::EdgeProperty<bool> efeature_;
        SurfaceMesh::VertexProperty<bool> vlocked_;
        SurfaceMesh::EdgeProperty<bool> elocked_;
        SurfaceMesh::VertexProperty<float> vsizing_;

        SurfaceMesh::VertexProperty <vec3> refpoints_;
        SurfaceMesh::VertexProperty <vec3> refnormals_;
        SurfaceMesh::VertexProperty<float> refsizing_;
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_SURFACE_MESH_REMESHING_H