#include <module/algo/surface_mesh_topology.h>
#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_components.h>

namespace scan3d {


    SurfaceMeshTopology::SurfaceMeshTopology(const SurfaceMeshComponent *comp) {
        component_ = comp;
        SurfaceMesh *mesh = comp->mesh();

        number_of_borders_ = 0;
        largest_border_size_ = 0;

        auto is_marked = mesh->add_halfedge_property<bool>("SurfaceMeshTopology:is_marked");

        for (auto h : comp->halfedges()) {
            auto cur = h;
            if (mesh->is_border(cur) && !is_marked[cur]) {
                ++number_of_borders_;
                std::size_t border_size = 0;
                do {
                    ++border_size;
                    is_marked[cur] = true;
                    cur = mesh->next(cur);
                } while (cur != h);
                largest_border_size_ = std::max(largest_border_size_, border_size);
            }
        }

        mesh->remove_halfedge_property(is_marked);
    }


    int SurfaceMeshTopology::euler_poincare() const {
        return component_->n_vertices() + component_->n_faces() - component_->n_edges();
    }

}