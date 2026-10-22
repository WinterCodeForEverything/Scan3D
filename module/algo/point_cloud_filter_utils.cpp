#include <algorithm>
#include <numeric>
#include <module/core/point_cloud.h>
#include <module/kdtree/kdtree_search_flann.h>
#include<module/algo/point_cloud_filter_utils.h>

#include <module/util/stop_watch.h>

namespace scan3d
{
	void  FilterUtils::RemoveStatisticalOutliers(PointCloud* cloud, int nbPoints, double stdRatio)
	{
		if (!cloud) {
			LOG(ERROR) << "empty input point cloud";
			return;
		}

		StopWatch w;
		w.start();
		LOG(INFO) << "building kd_tree...";//建立kdtree
		KdTreeSearch_FLANN kdtree;
		kdtree.begin();
		kdtree.add_point_cloud(cloud);
		kdtree.end();
		LOG(INFO) << "done. " << w.time_string();

		const std::vector<vec3>& points = cloud->points();
		std::vector<double> avgDistances = std::vector<double>(points.size());
		std::vector<vec3> indices;
		size_t validDistances = 0;

		w.restart();
		LOG(INFO) << "start statistic filter...";
		for (int i = 0; i < int(points.size()); i++)
		{
			std::vector<int> tmpIndices;
			std::vector<float> dist;
			kdtree.find_closest_k_points(points[i], nbPoints, tmpIndices, dist) ;//搜索k近邻
			std::vector<double> doubleVec(dist.begin(), dist.end());
			float mean = -1.0;
			if (dist.size() > 0u)
			{
				validDistances++;
				std::for_each(dist.begin(), dist.end(),
					[](float& d)
				{
					d = std::sqrt(d); 
				});
				mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();
			}
			avgDistances[i] = mean;
		}
		float cloudMean = std::accumulate(avgDistances.begin(), avgDistances.end(), 0.0,// 
			[](float const& x, float const& y)
		{
			return y > 0 ? x + y : x; 
		});
		cloudMean /= validDistances;
		float sqSum = std::inner_product(avgDistances.begin(), avgDistances.end(), avgDistances.begin(), 0.0,
			[](float const& x, float const& y)
		{
			return x + y;
		},
			[cloudMean](float const& x, float const& y)
		{
			return x > 0 ? (x - cloudMean) * (y - cloudMean) : 0;
		});
		// Bessel's correction
		float stdDev = std::sqrt(sqSum / (validDistances - 1));//计算标准差（根据标准差公式）
		float distanceThreshold = cloudMean + stdRatio * stdDev;//阈值为平均值加上标准差的乘数
		auto vertices = cloud->vertices();
		for (auto v : vertices)
		{
			if (avgDistances[v.idx()] < 0 || avgDistances[v.idx()] > distanceThreshold)//当某个点与其最近k个点的平均距离大于这个阈值时，判定该点为离群点并去除
			{
				cloud->delete_vertex(v);
			}
		}
		cloud->garbage_collection();
		LOG(INFO) << "done. " << w.time_string();
	}
}





