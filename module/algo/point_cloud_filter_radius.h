#ifndef SCAN3D_ALGO_POINT_CLOUD_FILTER_RADIUS_H
#define SCAN3D_ALGO_POINT_CLOUD_FILTER_RADIUS_H

namespace scan3d
{
	class PointCloud;
	class FilterRadius 
	{
	public:
		/// Use radius filter to remove outliers
		/// @param nbPoints: the number of neighboring points used to do filter.
		/// @param relative_searchRadius: the relative radius to search neighboring points, the radius relative to the model size. 
		static void RemoveRadiusOutliers(PointCloud* cloud, int nbPoints, float relative_searchRadius);
	};
}
#endif //SCAN3D_ALGO_POINT_CLOUD_FILTER_RADIUS_H