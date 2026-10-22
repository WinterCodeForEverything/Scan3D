#ifndef SCAN3D_ALGO_POISSON_RECONSTRUCTION_H
#define SCAN3D_ALGO_POISSON_RECONSTRUCTION_H


#include <string>


namespace scan3d {

    class PointCloud;
    class SurfaceMesh;

    class PoissonReconstruction {
    public:
        PoissonReconstruction(void);
        ~PoissonReconstruction(void);

        // parameters
        void set_depth(int d) { depth_ = d; }
        void set_sampers_per_node(float s) { samples_per_node_ = s; }

        // reconstruction
        SurfaceMesh *apply(const PointCloud *cloud, const std::string &density_attr_name = "v:density");

        // trimming
        static SurfaceMesh *trim(
                SurfaceMesh *mesh,
                const std::string &density_attr_name,
                float trim_value, float area_ratio, bool triangulate
        );

    public:
        // these parameters are usually not needed to be tuned，保持默认即可
        void set_full_depth(int v) { full_depth_ = v; }
        void set_voxel_depth_(int v) { voxelDepth_ = v; }
        void set_cg_depth(int v) { cgDepth_ = v; }
        void set_scale(float v) { scale_ = v; }
        void set_point_weight(float v) { pointWeight_ = v; }
        void set_gs_iter(int v) { gsIter_ = v; }
        void set_confidence(bool v) { confidence_ = v; }
        void set_normal_weight(bool v) { normalWeight_ = v; }
        void set_verbose(bool v) { verbose_ = v; }

    private:
        /*
        This integer is the maximum depth of the tree that will be used for surface
        reconstruction. Running at depth d corresponds to solving on a voxel grid
        whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the
        reconstructor adapts the octree to the sampling density, the specified
        reconstruction depth is only an upper bound.
        The default value for this parameter is 8.
        */
        int depth_;    // reconstruction depth

        /*
        This integer specifies the depth beyond depth the octree will be adapted.
        At coarser depths, the octree will be complete, containing all 2^d x 2^d x 2^d nodes.
        The default value for this parameter is 5.	*/
        int full_depth_; // adaptive octree depth

        /*
        This floating point value specifies the minimum number of sample points that should
        fall within an octree node as the octree construction is adapted to sampling density.
        For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more
        noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a
        smoother, noise-reduced, reconstruction. The default value is 1.0.
        */
        float samples_per_node_; // minimum number of samples

        bool triangulate_mesh_;

    private:
        // these parameters that usually do not need to change
        int voxelDepth_;
        int cgDepth_;
        float scale_;

        /*
        This floating point value specifies the importants that interpolation of the point
        samples is given in the formulation of the screened Poisson equation. The results
        of the original (unscreened) Poisson Reconstruction can be obtained by setting this
        value to 0. The default value for this parameter is 4.
        */
        float pointWeight_;    // interpolation weight

        int gsIter_;
        int threads_;
        bool confidence_;
        bool normalWeight_;
        bool verbose_;
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_POISSON_RECONSTRUCTION_H
