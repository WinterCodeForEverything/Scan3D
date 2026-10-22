#ifndef SCAN3D_ALGO_FILTER_UTILS_H
#define SCAN3D_ALGO_FILTER_UTILS_H

namespace scan3d
{ 
	class PointCloud;
	class FilterUtils //: public FilterUtils
	{
	public:
		/// Use statistic filter to remove outliers
		/// @param nbPoints: the number of neighboring points used to do filter.
		/// @param stdRatio: the radius to search neighboring points
		static void RemoveStatisticalOutliers(PointCloud* cloud, int nbPoints, double stdRatio );
	};
}

#endif  //SCAN3D_ALGO_FILTER_UTILS_H
