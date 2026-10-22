#ifndef SCAN3D_ALGO_GAUSSIAN_NOISE_H
#define SCAN3D_ALGO_GAUSSIAN_NOISE_H


namespace scan3d {

    class SurfaceMesh;
    class PointCloud;

    class GaussianNoise {
    public:

        /**
         * Add Gaussian noise (that has a normal distribution) to the surface mesh.
         * @param mesh The surface mesh.
         * @param sigma The standard deviation of the noise distribution. So about 68 percent of the noise values are
         *              within one standard deviation of the mean (mathematically, μ ± σ, where μ is the arithmetic
         *              mean), about 95 percent are within two standard deviations (μ ± 2σ).
         */
        static void apply(SurfaceMesh *mesh, float sigma);

        /**
        * Add Gaussian noise (that has a normal distribution) to a point cloud.
         * @param mesh The point cloud.
         * @param sigma The standard deviation of the noise distribution. So about 68 percent of the noise values are
         *              within one standard deviation of the mean (mathematically, μ ± σ, where μ is the arithmetic
         *              mean), about 95 percent are within two standard deviations (μ ± 2σ).
         */
        static void apply(PointCloud *cloud, float sigma);
    };

}


#endif  // SCAN3D_ALGO_GAUSSIAN_NOISE_H