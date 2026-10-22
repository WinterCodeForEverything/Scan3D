#include <vector>
#include<fstream>
#include <module/algo/point_cloud_feature_FPFH.h>
#include <module/core/point_cloud.h>
#include <module/util/logging.h>
#include<3rd_party/kd_tree/FLANN/flann.hpp>
#include <Eigen/Dense>



namespace scan3d
{
	using namespace Eigen;
	typedef flann::Index<flann::L2<double> > KDTree;
	class Feature {
	public:
		void Resize(int dim, int n) {
			data_.resize(dim, n);
			data_.setZero();
		}
		size_t Dimension() const { return data_.rows(); }
		size_t Num() const { return data_.cols(); }

	public:
		Eigen::MatrixXd data_;
	};


	Eigen::Vector4d ComputePairFeatures(const Eigen::Vector3d &p1,
		const Eigen::Vector3d &n1,
		const Eigen::Vector3d &p2,
		const Eigen::Vector3d &n2) {
		Eigen::Vector4d result;
		Eigen::Vector3d dp2p1 = p2 - p1;
		result(3) = dp2p1.norm();
		if (result(3) == 0.0) {
			return Eigen::Vector4d::Zero();
		}
		auto n1_copy = n1;
		auto n2_copy = n2;
		double angle1 = n1_copy.dot(dp2p1) / result(3);
		double angle2 = n2_copy.dot(dp2p1) / result(3);
		if (acos(fabs(angle1)) > acos(fabs(angle2))) {
			n1_copy = n2;
			n2_copy = n1;
			dp2p1 *= -1.0;
			result(2) = -angle2;
		}
		else {
			result(2) = angle1;
		}
		auto v = dp2p1.cross(n1_copy);
		double v_norm = v.norm();
		if (v_norm == 0.0) {
			return Eigen::Vector4d::Zero();
		}
		v /= v_norm;
		auto w = n1_copy.cross(v);
		result(1) = v.dot(n2_copy);
		result(0) = atan2(w.dot(n2_copy), n1_copy.dot(n2_copy));
		return result;
	}

	std::shared_ptr<Feature> ComputeSPFHFeature(
		Eigen::Matrix<double, Dynamic, 6> pc,
		KDTree &kdtree, int num, double d
	) {
		auto feature = std::make_shared<Feature>();
		feature->Resize(33, (int)pc.rows());
#pragma omp parallel for schedule(static)
		for (int i = 0; i < pc.rows(); i++) {

			flann::Matrix<double> dataset_mat(new double[1 * 3], 1, 3);

			for (int j = 0; j < 3; j++)
			{
				dataset_mat[0][j] = pc(i, j);
			}

			flann::SearchParams param(-1, 0.0);
			param.max_neighbors = num;

			std::vector<std::vector<int>> indices_vec;
			std::vector<std::vector<double>> dists_vec;
			int kkk = kdtree.radiusSearch(dataset_mat, indices_vec, dists_vec, d, param);


			if (kkk > 1) {
				double hist_incr = 100.0 / (double)(indices_vec[0].size() - 1);
				for (size_t k = 1; k < indices_vec[0].size(); k++) {
					auto pf = ComputePairFeatures(pc.block(i, 0, 1, 3).transpose(), pc.block(i, 3, 1, 3).transpose(),
						pc.block(indices_vec[0][k], 0, 1, 3).transpose(),
						pc.block(indices_vec[0][k], 3, 1, 3).transpose());
					int h_index = (int)(floor(11 * (pf(0) + M_PI) / (2.0 * M_PI)));
					if (h_index < 0) h_index = 0;
					if (h_index >= 11) h_index = 10;
					feature->data_(h_index, i) += hist_incr;
					h_index = (int)(floor(11 * (pf(1) + 1.0) * 0.5));
					if (h_index < 0) h_index = 0;
					if (h_index >= 11) h_index = 10;
					feature->data_(h_index + 11, i) += hist_incr;
					h_index = (int)(floor(11 * (pf(2) + 1.0) * 0.5));
					if (h_index < 0) h_index = 0;
					if (h_index >= 11) h_index = 10;
					feature->data_(h_index + 22, i) += hist_incr;
				}
			}
		}
		return feature;
	}
	bool PointCloudFeatureFPFH::SaveFPFHFeature( const PointCloud* cloud, int max_num, double radius, const string path, int pointnum )
	{
		auto f = ComputeFPFHFeature(cloud, max_num, radius );
		if (f == NULL)
			return false;
		std::ofstream savef(path);
		if (savef.is_open())
		{
			int size = f->data_.cols();
			if (pointnum >= size) pointnum = size - 1;
			savef << f->data_.transpose().block(0, 0, pointnum, f->data_.rows());
			savef.close();
		}
		else
		{
			LOG(ERROR) << "[SaveFPFHFeature] Failed to open output file.";
			return false;
		}
		return true;
	}
	std::shared_ptr<Feature> PointCloudFeatureFPFH::ComputeFPFHFeature(
		const PointCloud* cloud, int max_num, double radius) {
		const std::vector<vec3> &points = cloud->points();
		auto vNormals = cloud->get_vertex_property<vec3>("v:normal");
		if (!vNormals)
		{
			LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has no normal.";
			return NULL;
		}
		const std::vector<vec3> & normals = vNormals.vector();
		if (points.size() != normals.size())
		{
			LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has invalid normal.";
			return NULL;
		}
		MatrixXd pc(points.size(), 6);
		for (int i = 0; i < points.size(); i++)
		{
			pc(i, 0) = points[i][0];
			pc(i, 1) = points[i][1];
			pc(i, 2) = points[i][2];

			pc(i, 3) = normals[i][0];
			pc(i, 4) = normals[i][1];
			pc(i, 5) = normals[i][2];
		}
		auto feature = std::make_shared<Feature>();
		feature->Resize(33, pc.rows());
		int rows, dim;
		rows = (int)pc.rows();
		dim = 3;

		flann::Matrix<double> dataset_mat(new double[rows*dim], rows, dim);
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < dim; j++)
			{
				dataset_mat[i][j] = pc(i, j);
			}
		}

		//KDTreeSingleIndexParams 为搜索最大叶子数
		KDTree * tree = new KDTree(dataset_mat, flann::KDTreeSingleIndexParams(10));
		tree->buildIndex();


		auto spfh = ComputeSPFHFeature(pc, *tree, max_num, radius);
#pragma omp parallel for schedule(static)
		for (int i = 0; i < (int)pc.rows(); i++) {

			flann::Matrix<double> dataset_mat(new double[1 * 3], 1, 3);
			for (int ii = 0; ii < 1; ii++)
			{
				for (int j = 0; j < 3; j++)
				{
					dataset_mat[ii][j] = pc(i, j);
				}
			}
			flann::SearchParams param(-1, 0.0);
			param.max_neighbors = max_num;
			std::vector<std::vector<int>> indices_vec;
			std::vector<std::vector<double>> dists_vec;
			int kkk = tree->radiusSearch(dataset_mat, indices_vec, dists_vec, radius, param);

			if (kkk > 1) {

				double sum[3] = { 0.0, 0.0, 0.0 };
				for (size_t k = 1; k < indices_vec[0].size(); k++) {
					// skip the point itself
					double dist = dists_vec[0][k];
					if (dist == 0.0) continue;
					for (int j = 0; j < 33; j++) {
						double val = spfh->data_(j, indices_vec[0][k]) / dist;
						sum[j / 11] += val;
						feature->data_(j, i) += val;
					}
				}
				for (int j = 0; j < 3; j++)
					if (sum[j] != 0.0) sum[j] = 100.0 / sum[j];
				for (int j = 0; j < 33; j++) {
					feature->data_(j, i) *= sum[j / 11];
					// The commented line is the fpfh function in the paper.
					// But according to PCL implementation, it is skipped.
					// Our initial test shows that the full fpfh function in the
					// paper seems to be better than PCL implementation. Further
					// test required.
					feature->data_(j, i) += spfh->data_(j, i);
				}
			}
		}
		return feature;
	}
}// namespace scan3d

