#ifndef POINT_CLOUD_BILATERAL_FILTER_H
#define POINT_CLOUD_BILATERAL_FILTER_H
#include<cstdlib>
#include <list>
#include <map>
#include <cmath>
#include <cassert>

#ifndef M_PI
#define M_PI 3.14159265
#endif 

namespace scan3d {

	class PointCloud;
	class KdTreeSearch_FLANN;

	class TPointCloudBilateralFilter
	{
	private:
		static void estimate_normal(const PointCloud *cloud, const KdTreeSearch_FLANN &kdtree, std::vector<vec3>& normals);
		static void applyBilateralFilter(PointCloud* cloud, double radius, double normal_radius);

	public:

		/*Recycle bilateral filtering
		*@param radius  Radius of selected neighborhood point
		*@param normal_radius  The influence factor of the projection of the distance from the data point pi to the adjacent point in its normal direction on the data point pi
		*/
		static void applyBilateralFilterIteration(PointCloud* cloud, double radius, double normal_radius, const int interation = 3 );
	};
}
#endif //POINT_CLOUD_BILATERAL_FILTER_H