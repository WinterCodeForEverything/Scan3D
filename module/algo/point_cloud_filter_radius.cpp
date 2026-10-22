#include <algorithm>
#include <numeric>
#include <module/core/point_cloud.h>
#include <module/kdtree/kdtree_search_flann.h>
#include <module/algo/point_cloud_filter_radius.h>

#include <module/util/stop_watch.h>
#include<module/util/logging.h>

namespace scan3d
{
	void FilterRadius::RemoveRadiusOutliers(PointCloud* cloud, int nbPoints, float relative_searchRadius)
	{
		if (nbPoints < 1 || relative_searchRadius < 0 || relative_searchRadius > 1)
		{
			LOG(ERROR) << "Illegal input parameters,number of points or relative radius must be between 0 and 1";
			return;
		}
		if (!cloud) {
			LOG(ERROR) << "empty input point cloud";
			return;
		}
		float diagonal = cloud->bounding_box().diagonal();
		float searchRadius = diagonal * relative_searchRadius;
		StopWatch w;
		w.start();
		LOG(INFO) << "building kd_tree...";
		KdTreeSearch_FLANN kdtree;
		kdtree.begin();
		kdtree.add_point_cloud(cloud);
		kdtree.end();
		LOG(INFO) << "done. " << w.time_string();

		std::vector<vec3>& points = cloud->points();
		std::vector<bool> mask = std::vector<bool>(points.size());
		std::vector<int> tmpIndices;
		std::vector<float> dist;
		auto vertices = cloud->vertices();

		w.restart();
		LOG(INFO) << "start radius filter...";
		for (int i = 0; i < int(points.size()); i++)
		{
			tmpIndices.clear();
			dist.clear();

			//tmpIndices : The index of the nearest neighbor point searched in the point cloud,
			//searchRadius :the square value of the distance between the nearest neighbor point searched by dist and the queried point 
			//points[i]: the queried point
			kdtree.find_points_in_range(points[i], searchRadius, tmpIndices, dist);
			mask[i] = (tmpIndices.size() < nbPoints);
		}
		for (auto v : vertices)
		{
			if (mask[v.idx()])
				cloud->delete_vertex(v);
		}
		cloud->garbage_collection();
		LOG(INFO) << "done. " << w.time_string();
	}

}

