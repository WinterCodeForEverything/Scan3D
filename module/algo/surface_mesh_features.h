#ifndef SCAN3D_ALGO_SURFACE_MESH_FEATURES_H
#define SCAN3D_ALGO_SURFACE_MESH_FEATURES_H

#include <module/core/surface_mesh.h>


namespace scan3d {

    //! Detect and mark feature edges based on boundary or dihedral angle
    class SurfaceMeshFeatures {
    public:
        //! Construct with mesh to be analyzed.
        SurfaceMeshFeatures(SurfaceMesh *mesh);

        //! Clear features.
        void clear();

        //! Mark all boundary edges as features.
        void detect_boundary();

        //! Mark edges with dihedral angle larger than \p angle as feature.
        void detect_angle(float angle);

    private:
        SurfaceMesh *mesh_;

        SurfaceMesh::VertexProperty<bool> vfeature_;
        SurfaceMesh::EdgeProperty<bool> efeature_;
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_SURFACE_MESH_FEATURES_H