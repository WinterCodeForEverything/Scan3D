#ifndef SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H
#define SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include "registration/registration_PPF_Match.h"

namespace scan3d
{
	struct registration_parameter{
		double down_sampling_rate;//指点云体素网格化过程中体素的边长大小，是点云包围盒的边长的相对比。推荐范围0.02-0.08。
		double PPF_distance;// PPF离散的距离尺度，推荐值0.01-0.04
		double PPF_angle;//PPF离散的角度尺度，推荐值60-90。
		double referencepoint_rate;//参考点选择比例，推荐值0.05-0.2。

		double noise_bin_rate;//下采样体素步长：指点云体素化过程中体素的边长大小，是点云包围盒的最大边长的相对比。推荐范围0.1-0.05，点云质量越高该值越低。
		double noise_max_rate;//噪音分布比：基于有效点云在体素内的分布点数是相近，当体素内点云数量小于给定点数时则被认定为离群点。该定点数计算方法为体素内最大点数乘以噪音分布比，噪音分布比推荐值为0.2-0.5，值越大去除离群点效果越明显。
		
		double normal_bin_rate;//下采样体素步长：指点云体素网格化过程中体素的边长大小，是点云包围盒的边长的相对比。推荐范围0.01-0.002，点云质量越高该值越低。
		double normal_noise_max_rate;// 噪音分布比：基于有效点云在体素内的分布点数是相近，当体素内点云数量小于给定点数时则被认定为离群点。该点数计算方法为体素内最大点数乘以噪音分布比，噪音分布比推荐值为0.2-0.5，值越大去除离群点效果越明显。
		double normal_angle;//有效角度：由于测量物体表面法向与相机坐标系z轴的夹角越接近90度，测量精度越低，我们方法对估计夹角大于给定角度（有效角度）点直接删除。推荐值60-85。
		double normal_direcation;//法向方向：根据给定相机坐标系z方向调整点云法向方向，设置值与点云数据有关。
		double g_optimization_bin_rate;//下采样体素步长：指点云下采样中体素网格的边长大小，是点云包围盒的边长的相对比。推荐范围0.01-0.02。
		int  g_optimization_times;//优化迭代次数，推荐30-150。

		double turntable_voxel_size;//下采样体素步长：指点云体素网格化过程中体素的边长大小，
		double turntable_effective_area;//用于ICP重叠区域的收索
		double turntable_rotation_angle;//有效旋转角度，判断变换矩阵的正确性
		double turntable_translation_distance;//有效平移距离，判断变换矩阵的正确性
	};
	class PointCloud;
	class PointCloudRegistration 
	{
	public:
		/// 功能：实现将第一帧点云与其他帧点云配准，主要用于求取模型与点云之间的变换矩阵，也可用在模型上添加补充点云
		/// @参数 cloud: 输入所要配准的点云列表，要求点云与第一帧有重叠。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration_fine(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		/// 功能：实现将序列点云之间的配准，相邻点云配准，主要利用了粗配准和icp精配准，并进行了全局优化
		/// @参数 cloud: 输入所要配准的点云列表，要求点云是有重叠的序列。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		/// 功能：实现将序列点云之间的配准，相邻点云配准，只icp精配准，同意进行进行了全局优化
		/// @参数 cloud: 输入所要配准的点云列表，要求点云是有重叠的序列。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration_ICP_Princeton(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		
		static bool savePoses(const std::vector<std::string> &poseName, const std::vector<Matrix4d> &pose, const std::string filepath);
	};
}
#endif //SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H