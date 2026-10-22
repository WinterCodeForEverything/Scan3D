#ifndef SCAN3D_ALGO_POINT_CLOUD_SIMPLIFICATION_H
#define SCAN3D_ALGO_POINT_CLOUD_SIMPLIFICATION_H


#include <vector>

#include <module/core/point_cloud.h>


namespace scan3d {

    class KdTreeSearch;

    class PointCloudSimplification {
    public:

        /**
         * Query the average spacing of a point cloud.
         * @param cloud The point cloud.
         * @param k The number of nearest points used.
         * @param accurate True to use every point to get an accurate calculation; false to obtain aa approximate
         *                 measure, which uses only a subset (i.e., less than samples) of the points.
         * @param samples  Use less than this number of points for the calculation.
         * @param kdtree   A kdtree defined on this point cloud. If null, a new kdtree will be built and used.
         * @return The average spacing of the point clouds.
         */
        static float
        average_spacing(PointCloud *cloud, KdTreeSearch *kdtree = nullptr, int k = 6, bool accurate = false,
                        int samples = 200000
        );

        //----- simplification using a grid (non-uniform) ------------------------------------------------

        /**
         * Simplification of a point cloud using a regular grid covering the bounding box of the points. Simplification
         * is done by keeping a representative point (chosen arbitrarily) for each cell of the grid. This is non-uniform
         * simplification since the representative point is chosen arbitrarily.
         * @param cloud The point cloud.
         * @param cell_size The size of the cells of the grid.
         * @return The indices of points to be deleted.
         */
        static std::vector<PointCloud::Vertex> grid_simplification(PointCloud *cloud, float cell_size);

        //----- uniform simplification (specifying distance threshold) ------------------------------------

        /**
         * @brief Uniformly downsample a point cloud based on a distance criterion. This function can also be used for
         *        removing duplicated points of a point cloud.
         * @param cloud: The point cloud.
         * @param epsilon: The minimum allowed distance between points. Two points with a distance smaller than this
         *                 value are considered identical. After simplification, the distance of any point pair is
         *                 larger than this value.
         * @param kdtree   A kdtree defined on this point cloud. If null, a new kdtree will be built and used.
         * @return The indices of points to be deleted.
         */
        static std::vector<PointCloud::Vertex>
        uniform_simplification(PointCloud *cloud, float epsilon, KdTreeSearch *kdtree = nullptr);

        //----- uniform simplification (specifying expected point number) ---------------------------------

        /**
         * @brief Uniformly downsample a point cloud given the expected point number.
         * @param cloud: The point cloud.
         * @param num:   The expected point number, which must be less than or equal to the original point number.
         * @return The indices of points to be deleted.
         */
        static std::vector<PointCloud::Vertex> uniform_simplification(PointCloud *cloud, unsigned int num);
    };


} // namespace scan3d


#endif  // SCAN3D_ALGO_POINT_CLOUD_SIMPLIFICATION_H

