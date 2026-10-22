#include <module/algo_ext/duplicated_faces.h>
#include <module/algo_ext/self_intersection.h>

#include <module/algo_ext/mesh_surfacer.h>

#include <module/util/logging.h>


namespace scan3d {


    MeshSurfacer::MeshSurfacer()
    {
    }


    MeshSurfacer::~MeshSurfacer() {
    }


    std::vector< std::pair<SurfaceMesh::Face, std::vector<SurfaceMesh::Face> > >
    MeshSurfacer::detect_duplicated_faces(SurfaceMesh* mesh, bool exact, double dist_threshold)
    {
        DuplicatedFaces rdf;
        return rdf.detect(mesh, exact, dist_threshold);
    }


    unsigned int MeshSurfacer::remove_duplicated_faces(SurfaceMesh* mesh, bool exact, double dist_threshold)
    {
        DuplicatedFaces rdf;
        return rdf.remove(mesh, exact, dist_threshold);
    }


    std::vector< std::pair<SurfaceMesh::Face, SurfaceMesh::Face> >
    MeshSurfacer::detect_self_intersections(SurfaceMesh* mesh)
    {
        SelfIntersection msi;
        return msi.detect(mesh, false);
    }


    SurfaceMesh* MeshSurfacer::remesh_self_intersections(SurfaceMesh* mesh, bool stitch /* = true */)
    {
        SelfIntersection msi;
        return msi.remesh(mesh, stitch);
    }

}
