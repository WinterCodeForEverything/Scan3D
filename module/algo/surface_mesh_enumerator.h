#ifndef SCAN3D_ALGO_SURFACE_MESH_ENUMERATOR_H
#define SCAN3D_ALGO_SURFACE_MESH_ENUMERATOR_H


#include <module/core/surface_mesh.h>


namespace scan3d {

    class SurfaceMeshEnumerator {
    public:

        /**
         * Propagates a connected component of a surface mesh from the seed vertex.
         * @param mesh The input mesh.
         * @param id The vertex property storing the result.
         * @param seed The seed vertex from which the connected component will be propagated.
         * @param cur_id The index of the connected component to be propagated.
         * @attention The vertex property id must be initialize with "-1".
         */
        static void
        propagate_connected_component(SurfaceMesh *mesh, SurfaceMesh::VertexProperty<int> id, SurfaceMesh::Vertex seed,
                                      int cur_id);

        /**
         * Propagates a connected component of a surface mesh from the seed face.
         * @param mesh The input mesh.
         * @param id The face property storing the result.
         * @param seed The seed face from which the connected component will be propagated.
         * @param cur_id The index of the connected component to be propagated.
         * @attention The face property id must be initialize with "-1".
         */
        static void
        propagate_connected_component(SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id, SurfaceMesh::Face seed,
                                      int cur_id);

        /**
         * Enumerates the connected components of a surface mesh from its vertices.
         * @param mesh The input mesh.
         * @param id The vertex property storing the result.
         * @return The number of connected components.
         */
        static int enumerate_connected_components(SurfaceMesh *mesh, SurfaceMesh::VertexProperty<int> id);

        /**
         * Enumerates the connected components of a surface mesh from its faces.
         * @param mesh The input mesh.
         * @param id The face property storing the result.
         * @return The number of connected components.
         */
        static int enumerate_connected_components(SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id);

    };

}   // namespace scan3d


#endif  // SCAN3D_ALGO_SURFACE_MESH_ENUMERATOR_H

