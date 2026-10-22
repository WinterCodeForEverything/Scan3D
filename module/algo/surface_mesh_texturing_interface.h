#ifndef SCAN3D_ALGO_SURFACE_MESH_TEXTURING_INTERFACE_H
#define SCAN3D_ALGO_SURFACE_MESH_TEXTURING_INTERFACE_H

#include<string>
#define _USE_OPENCV
#include <opencv2/core.hpp>
#include <3rd_party/MVS/MVS/Interface.h>

using namespace std;
//using namespace MVS;
using namespace _INTERFACE_NAMESPACE;

namespace scan3d {

	//class SurfaceMesh;
    class SurfaceMeshTexturingInterface {
    public:
		SurfaceMeshTexturingInterface();
		~SurfaceMeshTexturingInterface();
		float GetNormalizationScale(uint32_t width, uint32_t height);

		//用于输入图片及相机参数
		bool ImportScene(const string& strWorkingFolder, const string& strRelativePose, Interface& scene);

		//将图片及相机参数保存成mvs，方便三方库调用
		bool saveCameraFile(const string& strWorkingFolder, const string& strRelativePose);

		//删除mvs文件
		void deleteCamera();
		
	public:
		string strCameraTmp;
		string strImageType;
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_SURFACE_MESH_TEXTURING_INTERFACE_H