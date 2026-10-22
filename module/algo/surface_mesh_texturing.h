#ifndef SCAN3D_ALGO_SURFACE_MESH_TEXTURING_H
#define SCAN3D_ALGO_SURFACE_MESH_TEXTURING_H

#include<string>


namespace scan3d {

	class SurfaceMesh;
    class SurfaceMeshTexturing {
    public:
		SurfaceMeshTexturing();
		~SurfaceMeshTexturing();


		bool texturing(SurfaceMesh* mesh,		//待贴图的网格
			const std::string& strWorkingFolder, //工作路径，需要包含照片文件夹和相机参数xml
			const std::string& strRelativePose, //相机外参txt的路径
			const std::string& strSaveFile, //保存贴图后模型和纹理图片路径、及保存的模型文件名
			const int nResolutionLevel, //降采样图像次数
			const int nMinResolution,	//最低图像分辨率，小于最小值不降采样
			const float fOutlierThreshold,	//用于判断离群面的阈值
			const float fRatioDataSmoothness,	//调整面的光滑程度
			const bool bGlobalSeamLeveling,		//全局缝隙平滑
			const bool bLocalSeamLeveling,		//局部缝隙平滑
			const int nTextureSizeMultiple,		//纹理重复次数
			const int nRectPackingHeuristic,	//调整贴图次数
			const int nColEmpty,	//背景颜色
			const int nMaxThreads,	//最大进程数
			const bool bImageType	//所用图片类型jpg or png
		);
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_SURFACE_MESH_TEXTURING_H