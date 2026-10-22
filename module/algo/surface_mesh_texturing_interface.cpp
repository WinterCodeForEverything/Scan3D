#include<vector>
#include <fstream>
#include<module/util/logging.h>


#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_texturing_interface.h>
#include <3rd_party/MVS/MVS/Common.h>

#define	IMAGEFOLDER	"/images/"
#define CAMERA_FILE	"/camera.xml"
#define CAMERA_FILESAVE	"/camera.mvs"



namespace scan3d {
	SurfaceMeshTexturingInterface::SurfaceMeshTexturingInterface()
	{
		// initialize global options
		Process::setCurrentProcessPriority((Process::Priority)-1);
#ifdef _USE_OPENMP
		if (OPT::nMaxThreads != 0)
			omp_set_num_threads(OPT::nMaxThreads);
#endif

#ifdef _USE_BREAKPAD
		// start memory dumper
		MiniDumper::Create(APPNAME, WORKING_FOLDER);
#endif

		Util::Init();
	}
	SurfaceMeshTexturingInterface::~SurfaceMeshTexturingInterface()
	{
		
	}

	float SurfaceMeshTexturingInterface::GetNormalizationScale(uint32_t width, uint32_t height) {
		return float(width > height ? width : height );
	}

	bool SurfaceMeshTexturingInterface::ImportScene(const string& strWorkingFolder, const string& strRelativePose, Interface& scene)
	{
		// read camera initial parameter
		string strCameraFile = strWorkingFolder + CAMERA_FILE;
		cv::FileStorage fs;
		fs.open(strCameraFile, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;
		//initial intrinsic parameter
		cv::Matx33d K;
		fs["camIntrinsics"] >> K;
		uint32_t iImageWidth = (int)fs["imageSize"]["Width"];
		uint32_t iImageHeight = (int)fs["imageSize"]["Height"];
		double fScale = (double)1 / GetNormalizationScale(iImageWidth, iImageHeight);
		K(0, 0) *= fScale;
		K(1, 1) *= fScale;
		K(0, 2) *= fScale;
		K(1, 2) *= fScale;
		//initial external parameter
		cv::Matx44d InitMatRC;
		fs["camExternals"] >> InitMatRC;
		fs.release();

		// read camera relative pose
		ifstream file(strRelativePose);
		if (!file.is_open())
			return false;
		vector<string> vPoseName;
		vector<cv::Matx44d> vRelativePose;
		while (!file.eof())
		{
			string line;
			getline(file, line);
			vPoseName.push_back(line);
			cv::Matx44d tmp;
			for (int row = 0; row < 4; row++)
				for (int col = 0; col < 4; col++)
				{
					float item = 0.0;
					file >> item;
					tmp(row, col) = item;
				}
			vRelativePose.push_back(tmp);
			getline(file, line);
		}
		file.close();
		assert(vRelativePose.size() == vPoseName.size());

		//compute camera external parameter and write parameters into scene
		cv::Matx41d initPose(InitMatRC(0, 3), InitMatRC(1, 3), InitMatRC(2, 3), 1);
		cv::Matx33d initR = InitMatRC.get_minor<3, 3>(0, 0);
		for (int i = 0; i < vPoseName.size()-1; i++)
		{
			Interface::Platform platform;
			Interface::Platform::Pose pose;
			Interface::Platform::Camera camera;
			camera.K = K;
			cv::Matx41d C = vRelativePose[i] * initPose; 
			camera.C = Interface::Pos3d(C(0, 0), C(1, 0), C(2, 0));
			camera.R = vRelativePose[i].get_minor<3, 3>(0, 0).inv()* initR;

			pose.R = Interface::Mat33d::eye();
			pose.C = Interface::Pos3d(0, 0, 0);

			platform.cameras.push_back(camera);
			platform.poses.push_back(pose);
			scene.platforms.push_back(platform);


			Interface::Image image;
			image.name = strWorkingFolder + IMAGEFOLDER + vPoseName[i] + strImageType;
			image.platformID = i;
			image.cameraID = 0;
			image.ID = i;
			image.poseID = (uint32_t)platform.poses.size() - 1;
			scene.images.push_back(image);
		}
		return true;
	}

	bool SurfaceMeshTexturingInterface::saveCameraFile(const string& strWorkingFolder, const string& strRelativePose)
	{
		if (strWorkingFolder.empty())
		{
			//LOG(ERROR) <<  "please input a camera file;";
			return false;
		}
		Interface scene;
		if (!ImportScene(strWorkingFolder, strRelativePose, scene))
		{
			//LOG(ERROR) <<  "error: fail to import camera file " << strInputName;
			return false;
		}
		strCameraTmp = strWorkingFolder +  CAMERA_FILESAVE;
		if (!ARCHIVE::SerializeSave(scene, strCameraTmp, 0))
		{
			//VERBOSE("error: fail to save file '%s'", fileout.c_str());
			return false;
		}
		return true;
	}

	void SurfaceMeshTexturingInterface::deleteCamera()
	{
		remove(MAKE_PATH_SAFE(strCameraTmp.c_str()));
	}
} // namespace scan3d