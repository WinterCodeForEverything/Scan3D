#ifndef SCAN3D_ALGO_SURFACE_MESH_COMPONENTS_H
#define SCAN3D_ALGO_SURFACE_MESH_COMPONENTS_H

#include <vector>

#include <module/core/surface_mesh.h>


namespace scan3d {

    /**
     * A connected component of a SurfaceMesh. Internally, it stores four lists of SurfaceMesh elements, i.e., vertices,
     * faces, edges, and halfedges.
    */
    class SurfaceMeshComponent {
    public:
        typedef SurfaceMesh::Face Face;
        typedef SurfaceMesh::Vertex Vertex;
        typedef SurfaceMesh::Edge Edge;
        typedef SurfaceMesh::Halfedge Halfedge;

    public:
        SurfaceMeshComponent(SurfaceMesh *mesh) : mesh_(mesh) {}

        /** Extracts connected components */
        static std::vector<SurfaceMeshComponent> extract(SurfaceMesh *mesh);

        /** Extracts a single connected component from the seed face */
        static SurfaceMeshComponent extract(SurfaceMesh *mesh, SurfaceMesh::Face seed);

        /** Extracts a single connected component from the seed vertex */
        static SurfaceMeshComponent extract(SurfaceMesh *mesh, SurfaceMesh::Vertex seed);

        const std::vector<Face> &faces() const { return faces_; }

        const std::vector<Vertex> &vertices() const { return vertices_; }

        const std::vector<Edge> &edges() const { return edges_; }

        const std::vector<Halfedge> &halfedges() const { return halfedges_; }

        std::size_t n_faces() const { return faces_.size(); }

        std::size_t n_vertices() const { return vertices_.size(); }

        std::size_t n_edges() const { return edges_.size(); }

        std::size_t n_halfedges() const { return halfedges_.size(); }

        /** Returns the surface mesh to which this component belongs. */
        SurfaceMesh *mesh() const { return mesh_; }

        /**
         * Returns the surface area of this component.
         * Internally it triangulates the face using the tessellator. So this method also works for concave faces.
         */
        float area() const;

        /** Returns the total border length of this component. */
        float border_length() const;

        /** Returns the bounding box of this component. */
        Box3 bbox() const;

        /** Translates this component by an offset vector. */
        void translate(const vec3 &offset);

        /** Constructs a surface mesh from this component. */
        SurfaceMesh *to_mesh() const;

    private:
        SurfaceMesh *mesh_;
        std::vector<Face> faces_;
        std::vector<Vertex> vertices_;
        std::vector<Halfedge> halfedges_;
        std::vector<Edge> edges_;
    };

}

#endif  // SCAN3D_ALGO_SURFACE_MESH_COMPONENTS_H

