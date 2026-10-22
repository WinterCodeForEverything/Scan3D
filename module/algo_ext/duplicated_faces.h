#ifndef SCAN3D_ALGO_DUPLICATED_FACES_H
#define SCAN3D_ALGO_DUPLICATED_FACES_H


#include <vector>

// Use this instead to mute errors resulting from bad CGAL assertions
//#define CGAL_KERNEL_NO_ASSERTIONS

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>	// Triangle triangle intersection
#include <CGAL/box_intersection_d.h>

#include <module/core/surface_mesh.h>


namespace scan3d {


    class DuplicatedFaces
    {
    public:
        DuplicatedFaces() {}
        ~DuplicatedFaces() {}

        // detect and return the duplicated faces
        // exact == true: do exact predict; otherwise use the distance threshold.
        // upon return, the second component of each entry contains the set of faces
        // duplicating the one stored as the first component.
        // TODO: implement using threshold (now only 'exact' is implemented).
        std::vector< std::pair<SurfaceMesh::Face, std::vector<SurfaceMesh::Face> > >
        detect(SurfaceMesh* mesh, bool exact = false, double dist_threshold = 1e-6);

        // detect and remove duplicated faces. Returns the number of deleted faces.
        // exact == true: do exact predict; otherwise use the distance threshold.
        unsigned int remove(SurfaceMesh* mesh, bool exact = false, double dist_threshold = 1e-6);

    private:
        typedef CGAL::Exact_predicates_exact_constructions_kernel	Kernel;

        typedef CGAL::Point_3<Kernel>		Point_3;
        typedef CGAL::Triangle_3<Kernel>	Triangle_3;

    private:
        struct Triangle {
        public:
            Triangle(const Point_3& a, const Point_3& b, const Point_3& c, SurfaceMesh::Face f) : triangle(a, b, c), face(f) {}
            Triangle_3 triangle;
            SurfaceMesh::Face face;
            std::vector<SurfaceMesh::Vertex> vertices;
        };

        // Axis-align boxes for all-pairs self-intersection detection
        typedef std::vector<Triangle>				Triangles;
        typedef typename Triangles::iterator		TrianglesIterator;
        typedef CGAL::Box_intersection_d::Box_with_handle_d<double, 3, TrianglesIterator>	Box;

    private:
        Triangles mesh_to_cgal_triangle_list(SurfaceMesh* esh);

        // test if two triangles duplicate
        bool do_duplicate(const Triangle& A, const Triangle& B, bool exact, double sqr_eps);

        Triangles triangle_faces_;
    };

}   // namespace scan3d

#endif  // SCAN3D_ALGO_DUPLICATED_FACES_H
