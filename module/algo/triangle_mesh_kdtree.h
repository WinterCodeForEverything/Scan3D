#ifndef SCAN3D_ALGO_TRIANGLE_MESH_KDTREE_H
#define SCAN3D_ALGO_TRIANGLE_MESH_KDTREE_H


#include <module/core/surface_mesh.h>
#include <vector>


namespace scan3d {

    //! A k-d tree for triangular surface meshes
    class TriangleMeshKdTree {
    public:
        //! construct with mesh
        TriangleMeshKdTree(const SurfaceMesh *mesh, unsigned int max_faces = 10, unsigned int max_depth = 30);

        ~TriangleMeshKdTree() { delete root_; }

        //! nearest neighbor information
        struct NearestNeighbor {
            float dist;
            SurfaceMesh::Face face;
            vec3 nearest;
            int tests;
        };

        //! Return handle of the nearest neighbor
        NearestNeighbor nearest(const vec3 &p) const;

    private:
        // triangle stores corners and face handle
        struct Triangle {
            Triangle() {}

            Triangle(const vec3 &x0, const vec3 &x1, const vec3 &x2, SurfaceMesh::Face ff) {
                x[0] = x0;
                x[1] = x1;
                x[2] = x2;
                f = ff;
            }

            vec3 x[3];
            SurfaceMesh::Face f;
        };

        // vector of Triangle
        typedef std::vector<Triangle> Triangles;

        // Node of the tree: contains parent, children and splitting plane
        struct Node {
            Node() : faces(nullptr), left_child(nullptr), right_child(nullptr) {};

            ~Node() {
                delete faces;
                delete left_child;
                delete right_child;
            }

            unsigned char axis;
            float split;
            Triangles *faces;
            Node *left_child;
            Node *right_child;
        };

        // Recursive part of build()
        unsigned int build_recurse(Node *node, unsigned int max_handles, unsigned int depth);

        // Recursive part of nearest()
        void nearest_recurse(Node *node, const vec3 &point, NearestNeighbor &data) const;

    private:
        Node *root_;
    };

} // namespace scan3d


#endif  // SCAN3D_ALGO_TRIANGLE_MESH_KDTREE_H