#include <module/algo/surface_mesh_enumerator.h>

#include <stack>


namespace scan3d {

    void SurfaceMeshEnumerator::propagate_connected_component(SurfaceMesh *mesh, SurfaceMesh::VertexProperty<int> id,
                                                              SurfaceMesh::Vertex seed, int cur_id) {
        std::stack<SurfaceMesh::Vertex> stack;
        stack.push(seed);

        while (!stack.empty()) {
            SurfaceMesh::Vertex top = stack.top();
            stack.pop();
            if (id[top] == -1) {
                id[top] = cur_id;
                for (auto v : mesh->vertices(top)) {
                    if (id[v] == -1) {
                        stack.push(v);
                    }
                }
            }
        }
    }


    int SurfaceMeshEnumerator::enumerate_connected_components(SurfaceMesh *mesh, SurfaceMesh::VertexProperty<int> id) {
        id.vector().assign(mesh->n_vertices(), -1);

        int cur_id = 0;
        for (auto v : mesh->vertices()) {
            if (id[v] == -1) {
                propagate_connected_component(mesh, id, v, cur_id);
                ++cur_id;
            }
        }
        return cur_id;
    }


    void SurfaceMeshEnumerator::propagate_connected_component(SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id,
                                                              SurfaceMesh::Face seed, int cur_id) {
        std::stack<SurfaceMesh::Face> stack;
        stack.push(seed);

        while (!stack.empty()) {
            SurfaceMesh::Face top = stack.top();
            stack.pop();
            if (id[top] == -1) {
                id[top] = cur_id;

                for (auto h : mesh->halfedges(top)) {
                    auto op = mesh->opposite(h);
                    if (!mesh->is_border(op)) {
                        auto f = mesh->face(op);
                        if (id[f] == -1) {
                            stack.push(f);
                        }
                    }
                }
            }
        }
    }


    int SurfaceMeshEnumerator::enumerate_connected_components(SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id) {
        id.vector().assign(mesh->n_faces(), -1);

        int cur_id = 0;
        for (auto f : mesh->faces()) {
            if (id[f] == -1) {
                propagate_connected_component(mesh, id, f, cur_id);
                ++cur_id;
            }
        }
        return cur_id;
    }

}