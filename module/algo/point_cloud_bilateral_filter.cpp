
#include<cstdlib>
#include <list>
#include <map>
#include <cmath>
#include <cassert>
#include <module/core/point_cloud.h>
#include <module/core/principal_axes.h>
#include <module/kdtree/kdtree_search_flann.h>
#include<module/algo/point_cloud_bilateral_filter.h>

#include <module/util/stop_watch.h>

namespace scan3d {

	void TPointCloudBilateralFilter::estimate_normal( const PointCloud *cloud, const KdTreeSearch_FLANN &kdtree, std::vector<vec3>& normals )
	{
		int num = cloud->n_vertices();
		const std::vector<vec3> &points = cloud->points();

#pragma omp parallel for
		for (int i = 0; i < num; ++i) {
			const vec3 &p = points[i];
			std::vector<int> neighbors;
			kdtree.find_closest_k_points( p, 16, neighbors );//搜索k近邻

			PrincipalAxes<3, float> pca;
			pca.begin();
			for (unsigned int j = 0; j < neighbors.size(); ++j) {
				int idx = neighbors[j];
				pca.add_point(points[idx]);
			}
			pca.end();

			normals[i] = pca.axis(2);
			if (normals[i].z < 0) // almost have positive Z
				normals[i] = -normals[i];
		}
	}

	void TPointCloudBilateralFilter::applyBilateralFilter( PointCloud* cloud, double radius, double normal_radius )
	{
		double diagonal = cloud->bounding_box().diagonal();
		radius = diagonal * radius;//以boundingbox的乘数为默认参数
		double m_weight = -0.5 / (radius * radius);
		double m_normal_weight = -0.5 / ( normal_radius * normal_radius );
		std::vector<vec3>& points = cloud->points();

		StopWatch w;
		w.start();
		LOG(INFO) << "building kd_tree..."; //建立kdtree
		KdTreeSearch_FLANN kdtree;
		kdtree.begin();
		kdtree.add_point_cloud(cloud);
		kdtree.end();
		LOG(INFO) << "done. " << w.time_string();

		w.restart();
		LOG(INFO) << "estimate normals...";
		std::vector<vec3> normals( cloud->n_vertices() );
		estimate_normal(cloud, kdtree, normals);//算法线
		LOG(INFO) << "done. " << w.time_string();

		w.restart();
		LOG(INFO) << "start bilateral filter...";
#pragma omp parallel for
		for (int i = 0; i < int(points.size()); i++)
		{
			std::vector<int> neighbors;
			std::vector<float> dist;
			kdtree.find_points_in_range(points[i], radius, neighbors, dist);//搜索紧邻点

			double nx, ny, nz;
			nx = normals[i][0];
			ny = normals[i][1];
			nz = normals[i][2];
			if (nx * points[i][0] + ny * points[i][1] + nz * points[i][2] < 0)//调整法线方向
			{
				nx = -nx;
				ny = -ny;
				nz = -nz;
			}

			double diffx, diffy, diffz;
			double  h, wc, ws;
			double sum = 0.0;
			double normalizer = 0.0;
			std::vector<float>::iterator itd = dist.begin();
			typename std::vector<int>::iterator it;
			for ( it = neighbors.begin(); it != neighbors.end(); ++it, ++itd )
			{
				diffx = points[*it][0] - points[i][0];
				diffy = points[*it][1] - points[i][1];
				diffz = points[*it][2] - points[i][2];
				h = nx * diffx + ny * diffy + nz * diffz;

				wc = exp(h * h * m_normal_weight);//数据点到临近点的距离在其法向上的投影对数据点的影响因子
				ws = exp((*itd) * m_weight);//每个数据点到其邻域点的距离对点的影响因子
				double dw = wc * ws;

				sum = sum + dw * h;
				normalizer = normalizer + dw;
			}
			if (normalizer < 1e-15)
			{
				sum = 0.0;
			}
			else
			{
				sum = sum / normalizer;
				points[i][0] += sum * nx;//每个点进行移动
				points[i][1] += sum * ny;
				points[i][2] += sum * nz;
			}
		}
		LOG(INFO) << "done. " << w.time_string();
	}

	void TPointCloudBilateralFilter::applyBilateralFilterIteration( PointCloud* cloud, double radius, double normal_radius, const int interation )
	{
		if (!cloud) {
			LOG(ERROR) << "empty input point cloud";
			return;
		}
		for (int i = 0; i < interation; i++)
		{
			LOG(INFO) << "Bilateral filter interation time: "<< i+1;
			applyBilateralFilter(cloud, radius, normal_radius);
		}
			
	}
}

