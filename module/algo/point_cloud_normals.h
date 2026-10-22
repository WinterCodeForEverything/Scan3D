#ifndef SCAN3D_ALGO_POINT_CLOUD_NORMALS_H
#define SCAN3D_ALGO_POINT_CLOUD_NORMALS_H

#include <string>


namespace scan3d {

    class PointCloud;

    class PointCloudNormals {
    public:
        /// Estimates the point cloud normals using PCA.
        /// @param k: the number of neighboring points to construct the covariance matrix.
        /// @param compute_curvature: also computes the curvature
        bool estimate(PointCloud *cloud, unsigned int k = 16, bool compute_curvature = false) const;

        /// Reorients the point cloud normals.
        /// This method implements the normal reorientation method described in
        /// Hoppe et al. Surface reconstruction from unorganized points. SIGGRAPH 1992.
        /// @param k: the number of neighboring points to construct the graph.
        bool reorient(PointCloud *cloud, unsigned int k = 16) const;

		bool invertOrientation(PointCloud *cloud) const;
    };


} // namespace scan3d


#endif  // SCAN3D_ALGO_POINT_CLOUD_NORMALS_H

