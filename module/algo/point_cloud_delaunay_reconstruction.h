#ifndef SCAN3D_ALGO_DELAUNAY_RECONSTRUCTION_H
#define SCAN3D_ALGO_DELAUNAY_RECONSTRUCTION_H


#include <string>


namespace scan3d {

    class PointCloud;
    class SurfaceMesh;

    class DelnaunayReconstruction {
    public:
		DelnaunayReconstruction();
		~DelnaunayReconstruction();
        // reconstruction
        SurfaceMesh *reconstruct(const PointCloud *cloud, //待网格化点云
									const float radius,		//搜寻半径，该值是相对于点云包围盒对角线长度的百分比
									const int nb_iterations,	//邻域迭代次数
									const int nb_neighbors,		//用于重建邻域点数
									const int nb_points,	//remesh保存点数
									const int nb_Lloyd_iter,  //Lloyd迭代优化次数(用于初始化具有更均匀分布的牛顿迭代)
									const int nb_Newton_iter, //牛顿迭代次数 
									const int Newton_m, //用于Hessian近似的评估数量
									const bool bRemeshing,	//是否进行重新网格化
									const bool bWaterTight	//判断模型是否闭合
			);
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_DELAUNAY_RECONSTRUCTION_H
