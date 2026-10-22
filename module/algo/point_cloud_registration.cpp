#include <module/algo/point_cloud_registration.h>
#include <module/core/point_cloud.h>
#include <module/util/logging.h>
#include <module/util/stop_watch.h>
#include<module/util/logging.h>
#include <module/util/file_system.h>
#include "registration/registration_ppf_helpers.hpp"
#include"registration/registration_PPF_Match.h"
#include<vector>

#include"ICP_Princeton/TriMesh.h"
#include"ICP_Princeton/TriMesh_algo.h"
#include"ICP_Princeton/ICP.h"
#include"ICP_Princeton/KDtree.h"

using namespace play3d;
using namespace ppf_match;
using namespace std;

namespace scan3d
{

	void PointCloudRegistration::Registration_ICP_Princeton(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		//trimesh::TriMesh *mesh1

		int pointnum = cloud.size();//获取点云帧数量
		std::vector<MatrixXf> pointcloud;//数据格式转化，保持点云数据
		pointcloud.resize(pointnum);
		std::vector<MatrixXf> oldpointcloud;//数据格式转化，保持点云数据
		oldpointcloud.resize(pointnum);
		clock_t tick1 = clock();
		pose.resize(pointnum);
		std::vector<Matrix4d>  playpose;
		playpose.resize(pointnum);
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			pose[point_each].setIdentity();
			playpose[point_each].setIdentity();

			std::vector<vec3> &points = cloud[point_each]->points();
			auto vNormals = cloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)//判断是含有法向，如果无法向，需压进行去噪和法向计算的处理
			{
				pointcloud[point_each].resize(points.size(), 3);
				for (int i = 0; i < points.size(); i++)
				{
					pointcloud[point_each](i, 0) = points[i][0];
					pointcloud[point_each](i, 1) = points[i][1];
					pointcloud[point_each](i, 2) = points[i][2];
				}
				//去噪
				MatrixXf r1 = BinningDenoise(pointcloud[point_each], para.noise_bin_rate, para.noise_max_rate);
				//法向估计
				MatrixXf r = BinningDenoiseNormal(r1, para.normal_bin_rate, para.normal_noise_max_rate, para.normal_angle);
				//是否需要进行法向方向改变
				if (para.normal_direcation == 1)
					r.block(0, 3, r.rows(), 3) = r.block(0, 3, r.rows(), 3)*-1;
				std::cout << cloud[point_each]->name() << std::endl;
				{
					//初始化配准后的点云数据
					PointCloud* p = new PointCloud;
					std::string name = file_system::base_name(cloud[point_each]->name());
					p->set_name(name + "_" + to_string(0));//点云名称设置
					std::vector<vec3> &points = p->points();
					auto normal = p->vertex_property<vec3>("v:normal");//法向
					auto color3 = p->vertex_property<vec3>("v:color");//颜色
					for (int i = 0; i < r.rows(); i++)
					{
						points.push_back(vec3(r(i, 0), r(i, 1), r(i, 2)));
						normal.vector().push_back(vec3(r(i, 3), r(i, 4), r(i, 5)));
						color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
					}
					p->resize(r.rows());
					std::cout << p->vertices_size() << std::endl;
					newcloud.push_back(p);
				}
				//处理后的点云数据
				pointcloud[point_each] = r;
			}
			else//判断是含有法向，如果有法向,则进行辅助处理
			{
				std::vector<vec3> & normals = vNormals.vector();
				{
					pointcloud[point_each].resize(points.size(), 6);
					//点云赋值
					for (int i = 0; i < points.size(); i++)
					{
						pointcloud[point_each](i, 0) = points[i][0];
						pointcloud[point_each](i, 1) = points[i][1];
						pointcloud[point_each](i, 2) = points[i][2];

						pointcloud[point_each](i, 3) = normals[i][0];
						pointcloud[point_each](i, 4) = normals[i][1];
						pointcloud[point_each](i, 5) = normals[i][2];
					}


					{//初始化配准后的点云数据

						PointCloud* p = new PointCloud;
						std::string name = file_system::base_name(cloud[point_each]->name());
						p->set_name(name + "_" + to_string(0));
						std::vector<vec3> &points = p->points();
						auto normal = p->vertex_property<vec3>("v:normal");
						auto color3 = p->vertex_property<vec3>("v:color");
						for (int i = 0; i < pointcloud[point_each].rows(); i++)
						{
							points.push_back(vec3(pointcloud[point_each](i, 0), pointcloud[point_each](i, 1), pointcloud[point_each](i, 2)));
							normal.vector().push_back(vec3(pointcloud[point_each](i, 3), pointcloud[point_each](i, 4), pointcloud[point_each](i, 5)));
							color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
						}
						p->resize(pointcloud[point_each].rows());
						newcloud.push_back(p);
					}

				}

			}




		}
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			oldpointcloud[point_each]=pointcloud[point_each];
		}
		clock_t tick2 = clock();
		std::cout << std::endl << "normal in "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << std::endl;

		
		std::vector <std::vector<std::vector<int>>> poseindex;
		poseindex.resize(1);
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			std::vector<int> a;
			a.push_back(point_each);
			poseindex[0].push_back(a);
		}

		ppf_model_parameter model_parameter;
		PPF_PSO_Match detectorpso(model_parameter);
		clock_t tick3 = clock();
		while (true)
		{
			int layer = poseindex.size() - 1;
			std::vector<std::vector<int>> newlayer;
			std::vector<int> pointindex;
			pointindex.push_back(0);
			for (int i = 0; i < poseindex[layer].size(); i++)
			{
				int i1 = poseindex[layer][i][0];
				int j = i + 1;
				if (j == poseindex[layer].size()) j = 0;
				int i2 = poseindex[layer][j][0];
				

				RowVector3f pc_max, pc_min, stepsize;
				for (int j = 0; j < 3; j++)
				{
					pc_max(j) = pointcloud[i1].col(j).maxCoeff();
					pc_min(j) = pointcloud[i1].col(j).minCoeff();
				}
				Vector2f  xrange, yrange, zrange;
				xrange(0) = pc_min(0);
				xrange(1) = pc_max(0);
				yrange(0) = pc_min(1);
				yrange(1) = pc_max(1);
				zrange(0) = pc_min(2);
				zrange(1) = pc_max(2);
				double d1 = (pc_max - pc_min).norm();
				MatrixXf dr1 = samplePCByQuantization(pointcloud[i1], xrange, yrange, zrange, d1*para.turntable_voxel_size, 0);
				//dr1 = pointcloud[i1];
				
				for (int j = 0; j < 3; j++)
				{
					pc_max(j) = pointcloud[i2].col(j).maxCoeff();
					pc_min(j) = pointcloud[i2].col(j).minCoeff();
				}
				xrange(0) = pc_min(0);
				xrange(1) = pc_max(0);
				yrange(0) = pc_min(1);
				yrange(1) = pc_max(1);
				zrange(0) = pc_min(2);
				zrange(1) = pc_max(2);
				double d2 = (pc_max - pc_min).norm();
				MatrixXf dr2 = samplePCByQuantization(pointcloud[i2], xrange, yrange, zrange, d2*para.turntable_voxel_size, 0);
				//dr2 = pointcloud[i2];
				
				double diameter = (d1 + d2) / 2.0*para.turntable_effective_area;
				//std::cout << "diameter= " << diameter << std::endl;
				MatrixXf  r1, r2;
				r1 = dr1;
				r2 = dr2;
				//std::cout << r2.rows() << " " << r1.rows() << std::endl;
				detectorpso.findsimilararea(dr1, dr2, r1, r2, diameter);

				std::cout << r2.rows() << " " << r1.rows() << std::endl;
				//Matrix4d pose1=detectorpso.ICP_Optimizing(pointcloud[i2], pointcloud[i1], 150);
				Matrix4d pose1;
				pose1.setIdentity();
				if(r2.rows()>50 && r1.rows()>50)
				   pose1 = detectorpso.ICP_Optimizing(r2, r1, 150);
				Vector3d   angle, translation;
				detectorpso.PoseMatrixToAngle(pose1, angle, translation);
				std::cout<< i << " " << angle.transpose() << " " << translation.norm() <<" "<< (d1 + d2) / 2.0*para.turntable_translation_distance<< std::endl 
					//<< pose1 
					<< std::endl;
				if (fabs(angle(0)) < para.turntable_rotation_angle 
					&& fabs(angle(1)) < para.turntable_rotation_angle
					&& fabs(angle(2)) < para.turntable_rotation_angle
					&& translation.norm()< (d1 + d2) / 2.0*para.turntable_translation_distance
					&& r2.rows() > 50 && r1.rows() > 50)
				{
					playpose[i2] = pose1;
					pointindex.push_back(i2);
				}
				else
				{
					newlayer.push_back(pointindex);
					pointindex.clear();
					if(j!=0)
					{
						pointindex.push_back(poseindex[layer][j][0]);
					}
				}
			}
			if(pointindex.size()>0)
			   newlayer.push_back(pointindex);
			//层的合并
			std::cout << "layer num=" << newlayer.size() << std::endl;
			for (int i = 0; i < newlayer.size(); i++)
			{
				std::cout << "layer id=" << i << std::endl;
				Matrix4d pose;
				pose.setIdentity();
				for (int j = 1; j < newlayer[i].size(); j++)
				{
					if (newlayer[i][j] != 0)
					{
						//std::cout << "  vvv=" << newlayer[i][j] << std::endl;
						pose = pose * playpose[newlayer[i][j]];
						MatrixXf  pc = transformPCPose(pointcloud[newlayer[i][j]], pose);
						int row1 = pointcloud[newlayer[i][0]].rows();
						int row2 = pc.rows();
						pointcloud[newlayer[i][0]].conservativeResize(row1 + row2, 6);
						pointcloud[newlayer[i][0]].block(row1, 0, row2, 6) = pc;
					}
					

				
					if (newlayer[i][j] == 0 && newlayer.size() > 1)
					{
						//std::cout << "  vv=" << newlayer[i][j] << std::endl;
						pose = pose * playpose[newlayer[i][j]];
						MatrixXf  pc = transformPCPose(pointcloud[newlayer[i][0]], pose.inverse());
						int row1 = pointcloud[newlayer[0][0]].rows();
						int row2 = pc.rows();
						pointcloud[newlayer[0][0]].conservativeResize(row1 + row2, 6);
						pointcloud[newlayer[0][0]].block(row1, 0, row2, 6) = pc;
						newlayer.resize(newlayer.size() - 1);
						//std::cout << " vv=" << newlayer.size() << std::endl;
					}
					
				}
			}
			if (poseindex[layer].size() == newlayer.size() || newlayer.size() == 1)
			{
				poseindex.push_back(newlayer);
				break;
			}
			poseindex.push_back(newlayer);
		}
		

		clock_t tick4 = clock();
		std::cout << std::endl << " Registration in "
			<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
			<< " sec" << std::endl;
		{
			/*int layer = poseindex.size() - 1;
			MatrixXf  tpc = pointcloud[poseindex[layer][0][0]];
			for (int i = 1; i < poseindex[layer].size(); i++)
			{
				int row1 = tpc.rows();
				int row2 = pointcloud[poseindex[layer][i][0]].rows();
				tpc.conservativeResize(row1 + row2, 6);
				tpc.block(row1, 0, row2, 6) = pointcloud[poseindex[layer][i][0]];
			}
			pointcloud[0] = tpc;
			for (int i = 1; i < pointnum; i++)
			{
				Matrix4d pose1 = detectorpso.ICP_Optimizing(oldpointcloud[1], tpc, 150);
				pose[i] = pose1*pose[i];
			}*/
		}
		/*for (int point_each = 0; point_each < pointnum; point_each++)
		{
			//点云变换
			pointcloud[point_each] = transformPCPose(oldpointcloud[point_each], pose[point_each]);
		}*/

		/*for (int point_each = 1; point_each < pointnum - 1; point_each++)
		{
			//更新姿态
			pose[point_each] = pose[point_each - 1] * pose[point_each];
		}
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//点云变换
			pointcloud[point_each] = transformPCPose(pointcloud[point_each], pose[point_each - 1]);
		}
		clock_t tick3 = clock();
		ppf_model_parameter model_parameter;
		PPF_PSO_Match detectorpso(model_parameter);

		std::vector<Matrix4d> icppose;//全局优化更新矩阵
		//全局优化
		detectorpso.GlobaloptimizationICP(pointcloud, para.g_optimization_bin_rate, para.g_optimization_times, icppose);
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//更新姿态
			pose[point_each - 1] = icppose[point_each] * pose[point_each - 1];
		}
		clock_t tick4 = clock();
		std::cout << std::endl << ".Globaloptimization in "
			<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
			<< " sec" << std::endl;*/
		std::cout << std::endl;
		std::cout << std::endl;
		//更新配准后的点云数据
		int layer = poseindex.size() - 1;
		newcloud.resize(poseindex[layer].size());
		for (int point_each = 0; point_each < poseindex[layer].size(); point_each++)
		{
			std::vector<vec3> &points = newcloud[point_each]->points();
			newcloud[point_each]->set_name("registration_" + to_string(point_each));//点云名称设置
			auto vNormals = newcloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has no normal.";
				return;
			}
			std::vector<vec3> & normals = vNormals.vector();
			if (points.size() != normals.size())
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has invalid normal.";
				return;
			}
			auto color3 = newcloud[point_each]->get_vertex_property<vec3>("v:color");//颜色
			MatrixXf r = pointcloud[poseindex[layer][point_each][0]];
			points.resize(r.rows());
			vNormals.vector().clear();
			color3.vector().clear();
			for (int i = 0; i < points.size(); i++)
			{
				points[i][0] = r(i, 0);
				points[i][1] = r(i, 1);
				points[i][2] = r(i, 2);
				vNormals.vector().push_back(vec3(r(i, 3), r(i, 4), r(i, 5)));
				color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 0.0 / 255));
			}
			newcloud[point_each] ->resize(r.rows());
		}

	}


	
	/*void PointCloudRegistration::Registration_ICP_Princeton(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		//trimesh::TriMesh *mesh1
		
		int pointnum = cloud.size();//获取点云帧数量
		std::vector<MatrixXf> pointcloud;//数据格式转化，保持点云数据
		pointcloud.resize(pointnum);
		clock_t tick1 = clock();
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = cloud[point_each]->points();
			auto vNormals = cloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)//判断是含有法向，如果无法向，需压进行去噪和法向计算的处理
			{
				pointcloud[point_each].resize(points.size(), 3);
				for (int i = 0; i < points.size(); i++)
				{
					pointcloud[point_each](i, 0) = points[i][0];
					pointcloud[point_each](i, 1) = points[i][1];
					pointcloud[point_each](i, 2) = points[i][2];
				}
				//去噪
				MatrixXf r1 = BinningDenoise(pointcloud[point_each], para.noise_bin_rate,para.noise_max_rate);
				//法向估计
				MatrixXf r = BinningDenoiseNormal(r1,para.normal_bin_rate,para.normal_noise_max_rate,para.normal_angle);
				//是否需要进行法向方向改变
				if(para.normal_direcation==1)
				      r.block(0, 3, r.rows(), 3) = r.block(0, 3, r.rows(), 3)*-1;
				std::cout << cloud[point_each]->name() << std::endl;
				{
					//初始化配准后的点云数据
					PointCloud* p = new PointCloud;
					std::string name = file_system::base_name(cloud[point_each]->name());
					p->set_name(name + "_" + to_string(0));//点云名称设置
					std::vector<vec3> &points = p->points();
					auto normal = p->vertex_property<vec3>("v:normal");//法向
					auto color3 = p->vertex_property<vec3>("v:color");//颜色
					for (int i = 0; i < r.rows(); i++)
					{
						points.push_back(vec3(r(i, 0), r(i, 1), r(i, 2)));
						normal.vector().push_back(vec3(r(i, 3), r(i, 4), r(i, 5)));
						color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
					}
					p->resize(r.rows());
					std::cout << p->vertices_size() << std::endl;
					newcloud.push_back(p);
				}
				//处理后的点云数据
				pointcloud[point_each] = r;
			}
			else//判断是含有法向，如果有法向,则进行辅助处理
			{
				std::vector<vec3> & normals = vNormals.vector();
				{
					pointcloud[point_each].resize(points.size(), 6);
					//点云赋值
					for (int i = 0; i < points.size(); i++)
					{
						pointcloud[point_each](i, 0) = points[i][0];
						pointcloud[point_each](i, 1) = points[i][1];
						pointcloud[point_each](i, 2) = points[i][2];

						pointcloud[point_each](i, 3) = normals[i][0];
						pointcloud[point_each](i, 4) = normals[i][1];
						pointcloud[point_each](i, 5) = normals[i][2];
					}

				
					{//初始化配准后的点云数据

						PointCloud* p = new PointCloud;
						std::string name = file_system::base_name(cloud[point_each]->name());
						p->set_name(name + "_" + to_string(0));
						std::vector<vec3> &points = p->points();
						auto normal = p->vertex_property<vec3>("v:normal");
						auto color3 = p->vertex_property<vec3>("v:color");
						for (int i = 0; i < pointcloud[point_each].rows(); i++)
						{
							points.push_back(vec3(pointcloud[point_each](i, 0), pointcloud[point_each](i, 1), pointcloud[point_each](i, 2)));
							normal.vector().push_back(vec3(pointcloud[point_each](i, 3), pointcloud[point_each](i, 4), pointcloud[point_each](i, 5)));
							color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
						}
						p->resize(pointcloud[point_each].rows());
						newcloud.push_back(p);
					}
					
				}
				
			}




		}

		clock_t tick2 = clock();
		std::cout << std::endl << "normal in "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << std::endl;

		std::vector<trimesh::TriMesh *> pointcloudmesh;//ICP数据格式转化
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			trimesh::TriMesh *mesh1 = new trimesh::TriMesh();//新建网格
			int n=pointcloud[point_each].rows();
			mesh1->vertices.resize(n);
			mesh1->normals.resize(n);
			for (int i = 0; i < n; i++)//对空间坐标和法向赋值
			{
				
				mesh1->vertices[i][0] = pointcloud[point_each](i,0);
				mesh1->vertices[i][1] = pointcloud[point_each](i, 1);
				mesh1->vertices[i][2] = pointcloud[point_each](i, 2);

				mesh1->normals[i][0] = pointcloud[point_each](i, 3);
				mesh1->normals[i][1] = pointcloud[point_each](i, 4);
				mesh1->normals[i][2] = pointcloud[point_each](i, 5);

			}
			pointcloudmesh.push_back(mesh1);
		}
		for (int point_each = 1; point_each < pointnum; point_each ++)//执行ICP配准
		{
			clock_t tick1 = clock();
			Matrix4d p;
			trimesh::ICP_xform_type xform_type = trimesh::ICP_RIGID;
			
			//构建查找树
			trimesh::KDtree *kd1 = new  trimesh::KDtree(pointcloudmesh[point_each - 1]->vertices);
			trimesh::KDtree *kd2 = new trimesh::KDtree(pointcloudmesh[point_each]->vertices);
			//ICP迭代权重
			vector<float> weights1, weights2;
			//参数初始化
			double m[16];
			m[0] = 1;  m[1] = 0;  m[2] =0;  m[3] = 0;
			m[4] = 0;  m[5] = 1;  m[6] = 0;  m[7] = 0;
			m[8] = 0;  m[9] = 0;  m[10] = 1; m[11] = 0;
			m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
			trimesh::xform xf1;
			trimesh::xform xf2;
			xf1 = m;
			xf2 = m;
			//ICP配准
			float err =ICP(pointcloudmesh[point_each - 1], pointcloudmesh[point_each], xf1, xf2, kd1, kd2, weights1, weights2,
				0.0f, 0, xform_type);
			
			//变换矩阵转化
			p(0,0) = xf2[0];
			p(0, 1) = xf2[1];
			p(0, 2) = xf2[2];
			p(0, 3) = xf2[3];

			p(1, 0) = xf2[4];
			p(1, 1) = xf2[5];
			p(1, 2) = xf2[6];
			p(1, 3) = xf2[7];

			p(2, 0) = xf2[8];
			p(2, 1) = xf2[9];
			p(2, 2) = xf2[10];
			p(2, 3) = xf2[11];

			p(3, 0) = xf2[12];
			p(3, 1) = xf2[13];
			p(3, 2) = xf2[14];
			p(3, 3) = xf2[15];

			delete kd1, kd2;
			//变换矩阵记录
			pose.push_back(p.transpose());

			clock_t tick4 = clock();
			std::cout << std::endl << "registration in "
				<< (double)(tick4 - tick1) / CLOCKS_PER_SEC
				<< " sec" << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;

		}
		for (int point_each = 1; point_each < pointnum - 1; point_each++)
		{
			//更新姿态
				pose[point_each] = pose[point_each - 1] * pose[point_each];
		}
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//点云变换
			pointcloud[point_each] = transformPCPose(pointcloud[point_each], pose[point_each - 1]);
		}
		clock_t tick3 = clock();
		ppf_model_parameter model_parameter;
		PPF_PSO_Match detectorpso(model_parameter);

		std::vector<Matrix4d> icppose;//全局优化更新矩阵
		//全局优化
		detectorpso.GlobaloptimizationICP(pointcloud, para.g_optimization_bin_rate, para.g_optimization_times, icppose);
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//更新姿态
			pose[point_each - 1] = icppose[point_each] * pose[point_each - 1];
		}
		clock_t tick4 = clock();
		std::cout << std::endl << ".Globaloptimization in "
			<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
			<< " sec" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		//更新配准后的点云数据
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = newcloud[point_each]->points();
			auto vNormals = newcloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has no normal.";
				return;
			}
			std::vector<vec3> & normals = vNormals.vector();
			if (points.size() != normals.size())
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has invalid normal.";
				return;
			}
			MatrixXf r = pointcloud[point_each];
			for (int i = 0; i < points.size(); i++)
			{
				points[i][0] = r(i, 0);
				points[i][1] = r(i, 1);
				points[i][2] = r(i, 2);

				normals[i][0] = r(i, 3);
				normals[i][1] = r(i, 4);
				normals[i][2] = r(i, 5);
			}
		}

	}
	*/
	void PointCloudRegistration::Registration(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
		ppf_model_parameter model_parameter;
		model_parameter.relativeSamplingStep = para.down_sampling_rate;
		model_parameter.relativeDistanceStep = para.PPF_distance;
		model_parameter.numAngles = para.PPF_angle;
		model_parameter.numplanes = 0;
		model_parameter.threshold_Angles = 25;
		model_parameter.thresholdlencur = 0.01;
		model_parameter.line_minlenght = 0.1;
		model_parameter.pointtoplanethreshold = 0.05;
		model_parameter.downsampleanglethreshold = 25;
		model_parameter.downsamplelevel = 1;

		//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
		ppf_scenematching_parameter scenematching_parameter;
		scenematching_parameter.minpairnum = 2;
		scenematching_parameter.voteAnglenum = 60;
		scenematching_parameter.relativeSceneDistance = 1;
		scenematching_parameter.targetposenumber = 50;

		scenematching_parameter.background = 0;//0为无

		scenematching_parameter.samplingrate = para.referencepoint_rate;
		scenematching_parameter.minoverlop = 0.0;
		scenematching_parameter.Clusteringdistance = 0.03;

		scenematching_parameter.overlopthreshold = para.down_sampling_rate;
		scenematching_parameter.angleoverlopthreshold = 15;

		scenematching_parameter.results_size = 1;
		scenematching_parameter.ICP_enable = 1;
		scenematching_parameter.ICP_nbIterations = 15;


		int pointnum = cloud.size();//获取点云帧数量
		std::vector<MatrixXf> pointcloud;//数据格式转化，保持点云数据
		pointcloud.resize(pointnum);
		clock_t tick1 = clock();
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = cloud[point_each]->points();
			auto vNormals = cloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)//判断是含有法向，如果无法向，需压进行去噪和法向计算的处理
			{
				pointcloud[point_each].resize(points.size(), 3);
				for (int i = 0; i < points.size(); i++)
				{
					pointcloud[point_each](i, 0) = points[i][0];
					pointcloud[point_each](i, 1) = points[i][1];
					pointcloud[point_each](i, 2) = points[i][2];
				}
				//去噪
				MatrixXf r1 = BinningDenoise(pointcloud[point_each], para.noise_bin_rate, para.noise_max_rate);
				//法向估计
				MatrixXf r = BinningDenoiseNormal(r1, para.normal_bin_rate, para.normal_noise_max_rate, para.normal_angle);
				//是否需要进行法向方向改变
				if (para.normal_direcation == 1)
					r.block(0, 3, r.rows(), 3) = r.block(0, 3, r.rows(), 3)*-1;
				std::cout << cloud[point_each]->name() << std::endl;
				{
					//初始化配准后的点云数据
					PointCloud* p = new PointCloud;
					std::string name = file_system::base_name(cloud[point_each]->name());
					p->set_name(name + "_" + to_string(0));//点云名称设置
					std::vector<vec3> &points = p->points();
					auto normal = p->vertex_property<vec3>("v:normal");//法向
					auto color3 = p->vertex_property<vec3>("v:color");//颜色
					for (int i = 0; i < r.rows(); i++)
					{
						points.push_back(vec3(r(i, 0), r(i, 1), r(i, 2)));
						normal.vector().push_back(vec3(r(i, 3), r(i, 4), r(i, 5)));
						color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
					}
					p->resize(r.rows());
					std::cout << p->vertices_size() << std::endl;
					newcloud.push_back(p);
					//continue;
				}
				//处理后的点云数据
				pointcloud[point_each] = r;
			}
			else//判断是含有法向，如果有法向,则进行辅助处理
			{
				std::vector<vec3> & normals = vNormals.vector();
				{
					pointcloud[point_each].resize(points.size(), 6);
					//点云赋值
					for (int i = 0; i < points.size(); i++)
					{
						pointcloud[point_each](i, 0) = points[i][0];
						pointcloud[point_each](i, 1) = points[i][1];
						pointcloud[point_each](i, 2) = points[i][2];

						pointcloud[point_each](i, 3) = normals[i][0];
						pointcloud[point_each](i, 4) = normals[i][1];
						pointcloud[point_each](i, 5) = normals[i][2];
					}


					{//初始化配准后的点云数据

						PointCloud* p = new PointCloud;
						std::string name = file_system::base_name(cloud[point_each]->name());
						p->set_name(name + "_" + to_string(0));
						std::vector<vec3> &points = p->points();
						auto normal = p->vertex_property<vec3>("v:normal");
						auto color3 = p->vertex_property<vec3>("v:color");
						for (int i = 0; i < pointcloud[point_each].rows(); i++)
						{
							points.push_back(vec3(pointcloud[point_each](i, 0), pointcloud[point_each](i, 1), pointcloud[point_each](i, 2)));
							normal.vector().push_back(vec3(pointcloud[point_each](i, 3), pointcloud[point_each](i, 4), pointcloud[point_each](i, 5)));
							color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
						}
						p->resize(pointcloud[point_each].rows());
						//std::cout << p->vertices_size() << std::endl;
						newcloud.push_back(p);
						//continue;
					}

				}
				
			}
				

			

		}
	
		//return;
		clock_t tick2 = clock();
		std::cout << std::endl << "point cloud  Pretreatment  in "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << std::endl;

		clock_t t_tick1 = clock();
		for (int point_each = 1; point_each < pointnum; point_each+=2)
		{
			clock_t tick1 = clock();
			//初始化配准方法类
			PPF_PSO_Match detectorpso(model_parameter);
			std::vector<MatrixXf> modelpoint;
			modelpoint.push_back(pointcloud[point_each]);
			//初始化配准方法模型的PPF特征哈希表化
			detectorpso.trainModel_PPF(modelpoint);
			clock_t tick2 = clock();

			std::cout << point_each+1<< "  frame trainModel in "
				<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
				<< " sec" << std::endl;

			clock_t tick3 = clock();
			double caltimme = 0;

			std::vector<int> inst_count;//输入的实例数量
			inst_count.push_back(1);
			std::vector<std::vector<SPose3D>> results;
			//模型的PPF配准
			detectorpso.match_PPF_MULTI_registration(pointcloud[point_each-1], results, inst_count, scenematching_parameter);
			pose.push_back(results[0][0].Pose);
			//std::cout << point_each-1 << std::endl << "overlap= "<< results[0][0].overlap<<"  "<< results[0][0].numVotes<< std::endl;
			if (point_each + 1 < pointnum)
			{
				std::vector<std::vector<SPose3D>> results;
				//模型的PPF配准
				detectorpso.match_PPF_MULTI_registration(pointcloud[point_each+1], results, inst_count, scenematching_parameter);
				pose.push_back(results[0][0].Pose);
				//std::cout << point_each +1<< std::endl << "overlap= "
				//	<< results[0][0].overlap << "  " << results[0][0].numVotes << std::endl;
			}
			clock_t tick4 = clock();
			//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
			if (point_each + 1 < pointnum)
			{
				std::cout << point_each + 1 << " and " << point_each + 2 << "  registration in "
					<< (double)(tick4 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;
			}
			else
			{
				std::cout << point_each + 1  << "  registration in "
					<< (double)(tick4 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;
			}
		
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;


		}
		//变换方法调整
		for (int point_each = 1; point_each < pointnum-1; point_each++)
		{
			if(point_each%2==1)
			   pose[point_each] = pose[point_each - 1] * pose[point_each].inverse();
			else
				pose[point_each] = pose[point_each - 1] * pose[point_each];
		}
		clock_t t_tick2 = clock();
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//点云变换
			pointcloud[point_each] = transformPCPose(pointcloud[point_each], pose[point_each - 1]);
		}
		
		std::cout << std::endl << "Average time for each pair of point cloud registration: "
			<< (double)(t_tick2 - t_tick1) / CLOCKS_PER_SEC/(pointnum-1)
			<< " sec" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;

		if (para.g_optimization_times > 0)
		{
			clock_t tick3 = clock();
			PPF_PSO_Match detectorpso(model_parameter);
			std::vector<Matrix4d> icppose;//全局优化更新矩阵
			//全局优化
			detectorpso.GlobaloptimizationICP(pointcloud, para.g_optimization_bin_rate, para.g_optimization_times, icppose);
			for (int point_each = 1; point_each < pointnum; point_each++)
			{
				//更新姿态
				//pose.push_back(icppose[point_each]);
				pose[point_each - 1] = icppose[point_each] * pose[point_each - 1];
			}

			clock_t tick4 = clock();
			std::cout << std::endl << ".Globaloptimization in "
				<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
				<< " sec" << std::endl;
			std::cout << std::endl;
			std::cout << std::endl;
		}
		
		
		//更新配准后的点云数据
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = newcloud[point_each]->points();
			auto vNormals = newcloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has no normal.";
				return;
			}
			std::vector<vec3> & normals = vNormals.vector();
			if (points.size() != normals.size())
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has invalid normal.";
				return;
			}
			MatrixXf r = pointcloud[point_each];
			for (int i = 0; i < points.size(); i++)
			{
				points[i][0] = r(i, 0) ;
				points[i][1] = r(i, 1) ;
				points[i][2] = r(i, 2);

				normals[i][0] = r(i, 3);
				normals[i][1] = r(i, 4);
				normals[i][2] = r(i, 5);
			}
		}
		//MatrixXf total_point = pointcloud[0];
	

	}
	/// 功能：实现将第一帧点云与其他帧点云配准，主要用于求取模型与点云之间的变换矩阵，也可用在模型上添加补充点云
	/// @参数 cloud: 输入所要配准的点云列表，要求点云与第一帧有重叠。 
	/// @参数 newcloud: 配准后的点云列表。
	/// @参数 para: 参数结构体。
	/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
	void PointCloudRegistration::Registration_fine(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
		ppf_model_parameter model_parameter;
		model_parameter.relativeSamplingStep = para.down_sampling_rate;
		model_parameter.relativeDistanceStep = para.PPF_distance;
		model_parameter.numAngles = para.PPF_angle;
		model_parameter.numplanes = 0;
		model_parameter.threshold_Angles = 25;
		model_parameter.thresholdlencur = 0.01;
		model_parameter.line_minlenght = 0.1;
		model_parameter.pointtoplanethreshold = 0.05;
		model_parameter.downsampleanglethreshold = 45;
		model_parameter.downsamplelevel = 1;

		//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
		ppf_scenematching_parameter scenematching_parameter;
		scenematching_parameter.minpairnum = 2;
		scenematching_parameter.voteAnglenum = 80;
		scenematching_parameter.relativeSceneDistance = 1;
		scenematching_parameter.targetposenumber = 50;

		scenematching_parameter.background = 0;//0为无

		scenematching_parameter.samplingrate = para.referencepoint_rate;
		scenematching_parameter.minoverlop = 0.0;
		scenematching_parameter.Clusteringdistance = 0.05;
		
		scenematching_parameter.overlopthreshold = para.down_sampling_rate;
		if (scenematching_parameter.overlopthreshold > 0.05)
			scenematching_parameter.overlopthreshold =0.05;
		scenematching_parameter.angleoverlopthreshold = 45;

		scenematching_parameter.results_size = 1;
		scenematching_parameter.ICP_enable = 1;
		scenematching_parameter.ICP_nbIterations = 10;


		int pointnum = cloud.size();//获取点云帧数量
		std::vector<MatrixXf> pointcloud;//数据格式转化，保持点云数据
		pointcloud.resize(pointnum);
		clock_t tick1 = clock();
		for (int point_each = 0; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = cloud[point_each]->points();
			auto vNormals = cloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)//判断是含有法向，如果无法向，需压进行去噪和法向计算的处理
			{
				pointcloud[point_each].resize(points.size(), 3);
				for (int i = 0; i < points.size(); i++)
				{
					pointcloud[point_each](i, 0) = points[i][0];
					pointcloud[point_each](i, 1) = points[i][1];
					pointcloud[point_each](i, 2) = points[i][2];
				}

				//去噪
				MatrixXf r1 = BinningDenoise(pointcloud[point_each], para.noise_bin_rate, para.noise_max_rate);
				//法向估计
				MatrixXf r = BinningDenoiseNormal(r1, para.normal_bin_rate, para.normal_noise_max_rate, para.normal_angle);
				//是否需要进行法向方向改变
				if (para.normal_direcation == 1)
					r.block(0, 3, r.rows(), 3) = r.block(0, 3, r.rows(), 3)*-1;
				std::cout << cloud[point_each]->name() << std::endl;
				{
					//初始化配准后的点云数据
					PointCloud* p = new PointCloud;
					std::string name = file_system::base_name(cloud[point_each]->name());
					p->set_name(name + "_" + to_string(0));//点云名称设置
					std::vector<vec3> &points = p->points();
					auto normal = p->vertex_property<vec3>("v:normal");//法向
					auto color3 = p->vertex_property<vec3>("v:color");//颜色
					for (int i = 0; i < r.rows(); i++)
					{
						points.push_back(vec3(r(i, 0), r(i, 1), r(i, 2)));
						normal.vector().push_back(vec3(r(i, 3), r(i, 4), r(i, 5)));
						color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
					}
					p->resize(r.rows());
					std::cout << p->vertices_size() << std::endl;
					newcloud.push_back(p);
				
				}
				//处理后的点云数据
				pointcloud[point_each] = r;
			}
			else//判断是含有法向，如果有法向,则进行辅助处理
			{
				std::vector<vec3> & normals = vNormals.vector();
				{
					pointcloud[point_each].resize(points.size(), 6);
					//点云赋值
					for (int i = 0; i < points.size(); i++)
					{
						pointcloud[point_each](i, 0) = points[i][0];
						pointcloud[point_each](i, 1) = points[i][1];
						pointcloud[point_each](i, 2) = points[i][2];

						pointcloud[point_each](i, 3) = normals[i][0];
						pointcloud[point_each](i, 4) = normals[i][1];
						pointcloud[point_each](i, 5) = normals[i][2];
					}


					{//初始化配准后的点云数据

						PointCloud* p = new PointCloud;
						std::string name = file_system::base_name(cloud[point_each]->name());
						p->set_name(name + "_" + to_string(0));
						std::vector<vec3> &points = p->points();
						auto normal = p->vertex_property<vec3>("v:normal");
						auto color3 = p->vertex_property<vec3>("v:color");
						for (int i = 0; i < pointcloud[point_each].rows(); i++)
						{
							points.push_back(vec3(pointcloud[point_each](i, 0), pointcloud[point_each](i, 1), pointcloud[point_each](i, 2)));
							normal.vector().push_back(vec3(pointcloud[point_each](i, 3), pointcloud[point_each](i, 4), pointcloud[point_each](i, 5)));
							color3.vector().push_back(vec3(255.0 / 255, 40.0 / 255, 255.0 / 255));
						}
						p->resize(pointcloud[point_each].rows());
						
						newcloud.push_back(p);
				
					}

				}

			}

			




		}

		clock_t tick2 = clock();
		std::cout << std::endl << "normal in "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << std::endl;
	

		tick1 = clock();
		//初始化配准方法类
		PPF_PSO_Match detectorpso(model_parameter);
		std::vector<MatrixXf> modelpoint;
		modelpoint.push_back(pointcloud[0]);
		//初始化配准方法模型的PPF特征哈希表化
		detectorpso.trainModel_PPF(modelpoint);
		tick2 = clock();

		std::cout << 0 << std::endl << "trainModel in "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << std::endl;

		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			
			clock_t tick3 = clock();
			double caltimme = 0;
			std::vector<int> inst_count;//输入的实例数量
			inst_count.push_back(1);
			std::vector<std::vector<SPose3D>> results;
			//模型的PPF配准
			detectorpso.match_PPF_MULTI_registration(pointcloud[point_each], results, inst_count, scenematching_parameter);
			pose.push_back(results[0][0].Pose);
			std::cout << std::endl << "overlap= "
				<< results[0][0].overlap << "  " << results[0][0].numVotes << std::endl;
			
			clock_t tick4 = clock();
			std::cout << std::endl << "match_PPF in "
				<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
				<< " sec" << std::endl;
	
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;
			std::cout << std::endl << std::endl;

		}
		
		for (int point_each = 0; point_each < pointnum - 1; point_each++)
		{
			//变换方法调整
			pose[point_each] = pose[point_each].inverse();
		}
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//点云变换
			pointcloud[point_each] = transformPCPose(pointcloud[point_each], pose[point_each - 1]);
		}
		clock_t tick3 = clock();
		std::vector<Matrix4d> icppose;//全局优化更新矩阵
		//全局优化
		detectorpso.GlobaloptimizationICP_fine(pointcloud, para.g_optimization_bin_rate,100, icppose);
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			//更新姿态
			pose[point_each - 1] = icppose[point_each]* pose[point_each - 1];
		}
		

		clock_t tick4 = clock();
		std::cout << std::endl << ".Globaloptimization in "
			<< (double)(tick4 - tick3) / CLOCKS_PER_SEC
			<< " sec" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		//更新配准后的点云数据
		for (int point_each = 1; point_each < pointnum; point_each++)
		{
			std::vector<vec3> &points = newcloud[point_each]->points();
			auto vNormals = newcloud[point_each]->get_vertex_property<vec3>("v:normal");
			if (!vNormals)
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has no normal.";
				return;
			}
			std::vector<vec3> & normals = vNormals.vector();
			if (points.size() != normals.size())
			{
				LOG(ERROR) << "[ComputeFPFHFeature] Failed because input point cloud has invalid normal.";
				return;
			}
			MatrixXf r = pointcloud[point_each];
			for (int i = 0; i < points.size(); i++)
			{
				points[i][0] = r(i, 0);
				points[i][1] = r(i, 1);
				points[i][2] = r(i, 2);

				normals[i][0] = r(i, 3);
				normals[i][1] = r(i, 4);
				normals[i][2] = r(i, 5);
			}
		}
		
	}

	
	bool PointCloudRegistration::savePoses(const std::vector<std::string> &poseName, const std::vector<Matrix4d> &pose, const std::string filepath)
	{
		std::ofstream file(filepath);
		if (!file.is_open())
			return false;
		assert(poseName.size() == pose.size());
		for (int i = 0; i < pose.size(); i++)
		{
			file << poseName[i] << std::endl;
			file << pose[i] << std::endl;
		}
		file.close();
		return true;
	}
}

