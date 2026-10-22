#ifndef SCAN3D_ALGO_SURFACE_MESH_TOPOLYGY_H
#define SCAN3D_ALGO_SURFACE_MESH_TOPOLYGY_H


#include <cstddef>

namespace scan3d {

    class SurfaceMeshComponent;

    class SurfaceMeshTopology {
    public:
        SurfaceMeshTopology(const SurfaceMeshComponent *comp);

        /**
         * returns the Euler-Poincare characteristic,
         *   1: a disc
         *   2: a sphere
         */
        int euler_poincare() const;

        /** returns 0 for a closed surface. */
        std::size_t number_of_borders() const { return number_of_borders_; }

        /** returns the number of edges in the largest border. */
        std::size_t largest_border_size() const { return largest_border_size_; }

        bool is_closed() const { return number_of_borders_ == 0; }

        bool is_sphere() const { return (number_of_borders() == 0) && (euler_poincare() == 2); }
        bool is_disc() const { return (number_of_borders() == 1) && (euler_poincare() == 1); }
        bool is_cylinder() const { return (number_of_borders() == 2) && (euler_poincare() == 0); }
        bool is_torus() const { return (number_of_borders() == 0) && (euler_poincare() == 0); }

    private:
        const SurfaceMeshComponent *component_;

        std::size_t number_of_borders_;
        std::size_t largest_border_size_;
    };

}

#endif  // SCAN3D_ALGO_SURFACE_MESH_TOPOLYGY_H

