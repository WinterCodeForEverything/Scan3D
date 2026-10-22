#ifndef SCAN3D_ALGO_POINT_CLOUD_FEATURE_FPFH
#define SCAN3D_ALGO_POINT_CLOUD_FEATURE_FPFH

#include <memory>
namespace scan3d {
	using namespace std;


	class PointCloud;
	class Feature;
	class PointCloudFeatureFPFH { 
	private:
		/// 计算点云的FPFH特征，返回值为对应点的FPFH特征列表
		/// @参数 cloud: 输入点云 
		/// @参数 max_num: 计算 FPFH的最大领域点数
		/// @参数 radius: 计算 FPFH的邻域搜索半径
		std::shared_ptr<Feature> ComputeFPFHFeature( const PointCloud* cloud, int max_num, double radius );
	public:
		/// 计算点云的FPFH特征并保存
		/// @参数 max_num: 计算 FPFH的最大领域点数
		/// @参数 radius: 计算 FPFH的邻域搜索半径
		/// @参数 path: 保存的文件路径 
		/// @参数 pointnum:  保存点的数量
		bool SaveFPFHFeature(const PointCloud* cloud, int max_num, double radius, const string path, int pointnum );
	};
}

#endif //SCAN3D_ALGO_POINT_CLOUD_FEATURE_FPFH

