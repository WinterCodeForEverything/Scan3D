#include "registration_PPF_Match.h"
#include "registration_hash_murmur.hpp"
#include "registration_IcpOptimizer.h"
#include<ctime>
#include <random>


#include"../ICP_Princeton/TriMesh.h"
#include"../ICP_Princeton/TriMesh_algo.h"
#include"../ICP_Princeton/ICP.h"
#include"../ICP_Princeton/KDtree.h"

using namespace std;

namespace play3d {
namespace ppf_match
{
	static const size_t PPF_LENGTH = 5;



    static bool vMatrix3dCompareS(std::vector<Matrix3d>& a, std::vector<Matrix3d>& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.size() > b.size());
	}


	static bool intlistCompare(std::vector<int>& a, std::vector<int>& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.size() > b.size());
	}

	// routines for assisting sort
	static bool pose3DPtrCompare(const Pose3DPtr& a, const Pose3DPtr& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a->numVotes > b->numVotes);
	}

	static bool pose3DPtrCompareS(const SPose3D & a, const SPose3D& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.numVotes > b.numVotes);
	}
	static bool pose3DPtrCompareover(const SPose3D & a, const SPose3D& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		
		return (a.overnum > b.overnum);
	}

	static bool pose3DPtrCompareoverlap(const Pose3DPtr& a, const Pose3DPtr& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a->overlap > b->overlap);
	}

	static bool pose3DPtrCompareoverlapS(const  SPose3D &a, const SPose3D &b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.overlap > b.overlap);
	}

	static int sortPoseClusters(const PoseCluster3DPtr& a, const PoseCluster3DPtr& b)
	{
		//CV_Assert(!a.empty() && !b.empty());
		return (a->numVotes > b->numVotes);
	}

	

	// quantize ppf and hash it for proper indexing
	static KeyType hashPPF(const Vector4d& f, const double AngleStep, const double DistanceStep)
	{
		/*Vector4i key(
			(int)(f[0] / AngleStep),
			(int)(f[1] / AngleStep),
			(int)(f[2] / AngleStep),
			(int)(f[3] / DistanceStep));*/
		int key[4];
		key[0] = (int)(f[0] / AngleStep);
		key[1] = (int)(f[1] / AngleStep);
		key[2] = (int)(f[2] / AngleStep);
		key[3] = (int)(f[3] / DistanceStep);
		KeyType hashKey = 0;
		//cout << key.transpose() << endl;
		murmurHash(&key[0], 4 * sizeof(int), 42, &hashKey);
		//cout << hashKey << endl;
		return hashKey;// hashKey;
	}

	static KeyType hashPPF_key(const Vector4i& f)
	{
		int key[4];
		key[0] = (int)(f[0] / 1.0);
		key[1] = (int)(f[1] / 1.0);
		key[2] = (int)(f[2] / 1.0);
		key[3] = (int)(f[3] / 1.0);
		KeyType hashKey = 0;
		//cout << key.transpose() << endl;
		murmurHash(&key[0], 4 * sizeof(int), 42, &hashKey);
		//cout << hashKey << endl;
		return hashKey;// hashKey;
	}


	static double computeAlpha(const Vector3d& p1, const Vector3d& n1, const Vector3d& p2)
	{
		Vector3d Tmg, mpt;
		Matrix3d R;
		double alpha;

		computeTransformRT(p1, n1, R, Tmg);
		mpt = Tmg + R * p2;
		alpha = atan2(-mpt[2], mpt[1]);

		if (alpha != alpha)
		{
			return 0;
		}

		if (sin(alpha)*mpt[2]<0.0)
			alpha = -alpha;

		return (-alpha);
	}
	static double computeAlphaRT(const Vector3d& p2, const  Matrix3d &R, const Vector3d &Tmg)
	{
		Vector3d  mpt;

		double alpha;
		mpt = Tmg + R * p2;
		alpha = atan2(-mpt[2], mpt[1]);

		if (alpha != alpha)
		{
			return 0;
		}

		if (sin(alpha)*mpt[2] < 0.0)
			alpha = -alpha;

		return (-alpha);
	}
	static double computeAlphaRT(const Vector3d& p1, const Vector3d& n1, const Vector3d& p2, Matrix3d &R, Vector3d &Tmg)
	{
		Vector3d  mpt;
		
		double alpha;

		computeTransformRT(p1, n1, R, Tmg);
		mpt = Tmg + R * p2;
		alpha = atan2(-mpt[2], mpt[1]);

		if (alpha != alpha)
		{
			return 0;
		}

		if (sin(alpha)*mpt[2] < 0.0)
			alpha = -alpha;

		return (-alpha);
	}

	

	//PPF_PSO_Match类
	PPF_PSO_Match::PPF_PSO_Match()
	{}
	PPF_PSO_Match::PPF_PSO_Match(ppf_model_parameter model_para)
	{
		sampling_step_relative = model_para.relativeSamplingStep;//采样步长
		distance_step_relative = model_para.relativeDistanceStep;//ppf离散化步长
		angle_step_relative = model_para.numAngles;//ppf离散角度数
		angle_step_radians = (360.0 / angle_step_relative)*M_PI / 180.0;//ppf角度离散步长
		//SceneSampleStep = 1.0/RelativeSceneSampleStep;
		angle_step = angle_step_radians;//离散步长

		modelplane.resize(1);
		modelplane[0] = model_para.numplanes;

		threshold_Angles = model_para.threshold_Angles*M_PI / 180.0;//角度阈值
		line_minlenght = model_para.line_minlenght;//最小距离阈值
		pointtoplanethreshold = model_para.pointtoplanethreshold;//点到平面距离
		downsamplelevel = model_para.downsamplelevel;//下采样层数
		downsampleanglethreshold = model_para.downsampleanglethreshold;//下采样角度 
	}
	PPF_PSO_Match::~PPF_PSO_Match()
	{}

	void PPF_PSO_Match::trainModel_PPF(const std::vector<MatrixXf> & original_model)
	{
		
		int model_sum = original_model.size();
		// 清楚前一次模型哈希表的数据
		if (model_para.model.size() > 0)
		{
			for (int model_id = 0; model_id < model_para.model.size(); model_id++)
			{
				hashtableDestroy(model_para.model[model_id].hash_table);
				model_para.model[model_id].ppf.swap(vector<int>());
			}
			model_para.model.swap(std::vector<PPF_PSO_MODEL>());
			model_para.scene.swap(std::vector<PPF_PSO_SCENE>());
		}
		//设置模型数量
		model_para.model.resize(model_sum);

		std::vector<Vector2f> xRangelist;
		std::vector<Vector2f> yRangelist;
		std::vector<Vector2f> zRangelist;

		for (int model_id = 0; model_id < model_sum; model_id++)
		{
			//模型平面数量
			model_para.model[model_id].model_Plane.clear();
			model_para.model[model_id].model_Plane.resize(0);
			//模型原始数据
			model_para.model[model_id].original_model = original_model[model_id];//
			//cout << original_model[model_id].maxCoeff() << original_model[model_id].minCoeff();
			//包围盒
			Vector2f xRange, yRange, zRange;
			computeBboxStd(model_para.model[model_id].original_model, xRange, yRange, zRange);
			xRangelist.push_back(xRange);
			yRangelist.push_back(yRange);
			zRangelist.push_back(zRange);
			// compute sampling step from diameter of bbox 计算包围盒直径和 采样半径
			float dx = xRange[1] - xRange[0];
			float dy = yRange[1] - yRange[0];
			float dz = zRange[1] - zRange[0];
			//模型中心点
			model_para.model[model_id].model_center(0) = (xRange[1] + xRange[0]) / 2.0;
			model_para.model[model_id].model_center(1) = (yRange[1] + yRange[0]) / 2.0;
			model_para.model[model_id].model_center(2) = (zRange[1] + zRange[0]) / 2.0;


			//模型直径
			model_para.model[model_id].model_diameter = sqrt(dx * dx + dy * dy + dz * dz);
			//参考点1
			model_para.model[model_id].model_referencepoint1(0) = model_para.model[model_id].model_center(0) - model_para.model[model_id].model_diameter;
			model_para.model[model_id].model_referencepoint1(1) = model_para.model[model_id].model_center(1);
			model_para.model[model_id].model_referencepoint1(2) = model_para.model[model_id].model_center(2);
			//参考点2
			model_para.model[model_id].model_referencepoint2(0) = model_para.model[model_id].model_center(0);
			model_para.model[model_id].model_referencepoint2(1) = model_para.model[model_id].model_center(1) - model_para.model[model_id].model_diameter;
			model_para.model[model_id].model_referencepoint2(2) = model_para.model[model_id].model_center(2);
			//参考点列表
			model_para.model[model_id].ferencepoint.resize(4, 3);
			model_para.model[model_id].ferencepoint.setOnes();
			model_para.model[model_id].ferencepoint.block(0, 0, 3, 1) = model_para.model[model_id].model_center;
			model_para.model[model_id].ferencepoint.block(0, 1, 3, 1) = model_para.model[model_id].model_referencepoint1;
			model_para.model[model_id].ferencepoint.block(0, 2, 3, 1) = model_para.model[model_id].model_referencepoint2;
			//模型下采样步长
			model_para.model[model_id].model_samplingdistance
				= (double)(model_para.model[model_id].model_diameter * sampling_step_relative);
			//cout << model_para.model[model_id].model_samplingdistance << " " << model_para.model[model_id].model_diameter << " " << sampling_step_relative << " " << endl;
			//模型ppf离散步长
			model_para.model[model_id].model_ppf_distance_step
				= (double)(model_para.model[model_id].model_diameter * distance_step_relative);
		}

		//endl

		for (int model_id = 0; model_id < model_sum; model_id++)
		{
			//边界探索下采样


			VectorXi sumnum_model;
		   //非均匀下采样
			model_para.model[model_id].downsample_model = samplePCByQuantization_normal(model_para.model[model_id].original_model,
				xRangelist[model_id], yRangelist[model_id], zRangelist[model_id],
				(float)model_para.model[model_id].model_samplingdistance, downsampleanglethreshold / 180 * 3.1415, downsamplelevel, sumnum_model);//无加权点云下采样
			//模型评价点云
			model_para.model[model_id].score_model = model_para.model[model_id].downsample_model;
			
			cout << "模型点数=" << model_para.model[model_id].downsample_model.rows() << endl;
			int  downsample_model_rows = model_para.model[model_id].downsample_model.rows();
			int size = downsample_model_rows * downsample_model_rows;//哈希表大小

			//相关数据初始化
			model_para.model[model_id].hash_table = hashtableCreate(size, NULL);//建立哈希表
			model_para.model[model_id].hash_nodes = (THash_MULTI*)calloc(size, sizeof(THash_MULTI));
			model_para.model[model_id].tMT.resize(downsample_model_rows);
			model_para.model[model_id].RMT.resize(downsample_model_rows);
			model_para.model[model_id].alpha.resize(size);
			double model_ppf_distance_step = model_para.model[model_id].model_ppf_distance_step;
			double model_minlenght = line_minlenght * model_para.model[model_id].model_diameter;
			for (int i = 0; i < downsample_model_rows; i++)
			{


				const Vector3d p1 = model_para.model[model_id].downsample_model.row(i).head(3).cast<double>();//获得坐标
				const Vector3d n1 = model_para.model[model_id].downsample_model.row(i).tail(3).cast<double>();//获得法向


				 //构建PPF特征哈希表;
				for (int j = 0; j < downsample_model_rows; j++)
				{
					if (i != j)
					{

						const Vector3d p2 = model_para.model[model_id].downsample_model.row(j).head(3).cast<double>();//获得坐标
						const Vector3d n2 = model_para.model[model_id].downsample_model.row(j).tail(3).cast<double>();//获得法向

						Vector4d f;
						computePPFFeatures(p1, n1, p2, n2, f);//计算特征值f
						//对一些不符合常规的特征去掉，主要为平面上奇异性大的点对
						if (f[3] > model_minlenght&& f[2] > 5.0 / 180 * 3.1415
							&& (f[0] > 95.0 / 180 * 3.1415 || f[0] < 85.0 / 180 * 3.1415))
						{
							//获得特征值
							KeyType hashValue = hashPPF(f, angle_step_radians, model_para.model[model_id].model_ppf_distance_step);//计算哈希表位置
							//用于检测PPF特征值是否一致
							unsigned long long n = ((unsigned long long)(f[3] / model_ppf_distance_step)) * 1000000000 +
								((unsigned long long)(f[0] / angle_step_radians)) * 1000000 +
								((unsigned long long)(f[1] / angle_step_radians)) * 1000 +
								((unsigned long long)(f[2] / angle_step_radians));		
							//角度、旋转和平移参数
							Matrix3d R;
							Vector3d Tmg;
							double alpha = computeAlphaRT(p1, n1, p2, R, Tmg);

							//点位置索引
							uint ppfInd = i * downsample_model_rows + j;

							//获得属性表的记录位置
							THash_MULTI * hashNode = &(model_para.model[model_id].hash_nodes[ppfInd]);

							hashNode->ppfindex = n;//特征转换为哈希表的关键值
							hashNode->i = i;//特征参考点
							hashNode->ppfInd = ppfInd;//特征点点对另一点在hash_nodes中的位置
							//插入结点
							hashtableInsertHashed(model_para.model[model_id].hash_table, hashValue, (void*)hashNode);
							//
							model_para.model[model_id].RMT[i] = R;//只有i点有关
							model_para.model[model_id].tMT[i] = Tmg;//只有i点有关
							model_para.model[model_id].alpha[ppfInd] = alpha;//根点对有关

						}
					}
				}
			}

		}


	}

    void PPF_PSO_Match::GlobaloptimizationICP_fine(std::vector<MatrixXf> &original_model, double downsamplestep, double icpiterationcout, std::vector<Matrix4d> &icppose)
	{
		/* 1.求体素并下采样
		   2.
		*/
		int modelsize = original_model.size();
		icppose.resize(modelsize);
		for (int model = 0; model < modelsize; model++)
		{
			icppose[model].setIdentity();

		}
		

		MatrixXf  minmax(1, 6);
		RowVector3f min, max;

		for (int model = 0; model < 1; model++)
		{
			minmax(model, 0) = original_model[model].col(0).minCoeff();
			minmax(model, 1) = original_model[model].col(1).minCoeff();
			minmax(model, 2) = original_model[model].col(2).minCoeff();
			minmax(model, 3) = original_model[model].col(0).maxCoeff();
			minmax(model, 4) = original_model[model].col(1).maxCoeff();
			minmax(model, 5) = original_model[model].col(2).maxCoeff();

			icppose[model].setIdentity();
		}
		{
			min(0) = minmax.col(0).minCoeff();
			min(1) = minmax.col(1).minCoeff();
			min(2) = minmax.col(2).minCoeff();

			max(0) = minmax.col(3).maxCoeff();
			max(1) = minmax.col(4).maxCoeff();
			max(2) = minmax.col(5).maxCoeff();
			double d = (min - max).norm();
			d = d * downsamplestep;
			//
			RowVector3i gridsum;
			for (int i = 0; i < 3; i++)
			{
				gridsum(i) = (max(i) - min(i)) / d + 1;
			}
			int totalsum = gridsum(0)*gridsum(1)*gridsum(2);

			std::vector<RowVector3f> point1;
			std::vector<RowVector3f> normal1;
			VectorXi pointsum1;
			point1.resize(totalsum);
			normal1.resize(totalsum);
			pointsum1.resize(totalsum);
			pointsum1.setZero();
			
			for (int i = 0; i < totalsum; i++)
			{
				point1[i].setZero();
				normal1[i].setZero();
			}

			//第一帧点云
			int n = original_model[0].rows();
			for (int i = 0; i < n; i++)
			{
				RowVector3f a = (original_model[0].row(i).head(3) - min) / d;
				int k = int(a(0))*gridsum(1)*gridsum(2) + int(a(1))*gridsum(2) + int(a(2));
				point1[k] += original_model[0].row(i).head(3);
				normal1[k] += original_model[0].row(i).tail(3);
				pointsum1(k) += 1;
			}

			std::vector<RowVector3f> pointr1;
			std::vector<RowVector3f> normalr1;

			for (int i = 0; i < totalsum; i++)
			{
				if (pointsum1(i) > 0)
				{
					double d = normal1[i].norm();
					if (d > 0.0000001)
					{
						pointr1.push_back(point1[i] / pointsum1(i));
						normalr1.push_back(normal1[i] / d);
					}

				}
				
			}
	
			trimesh::TriMesh *mesh1 = new trimesh::TriMesh();;
			mesh1->vertices.resize(pointr1.size());
			mesh1->normals.resize(pointr1.size());
			n = pointr1.size();
			for (int i = 0; i < n; i++)
			{
				mesh1->vertices[i][0] = pointr1[i](0);
				mesh1->vertices[i][1] = pointr1[i](1);
				mesh1->vertices[i][2] = pointr1[i](2);

				mesh1->normals[i][0] = normalr1[i](0);
				mesh1->normals[i][1] = normalr1[i](1);
				mesh1->normals[i][2] = normalr1[i](2);
			}
			trimesh::KDtree *kd1 = new  trimesh::KDtree(mesh1->vertices);

			for (int model =1; model <modelsize; model++)
			{
				trimesh::TriMesh *mesh2 = new trimesh::TriMesh();;

				mesh2->vertices.resize(original_model[model].rows());
				mesh2->normals.resize(original_model[model].rows());

				n = original_model[model].rows();
				for (int i = 0; i < n; i++)
				{
					mesh2->vertices[i][0] = original_model[model](i,0);
					mesh2->vertices[i][1] = original_model[model](i, 1);
					mesh2->vertices[i][2] = original_model[model](i, 2);

					mesh2->normals[i][0] = original_model[model](i, 3);
					mesh2->normals[i][1] = original_model[model](i, 4);
					mesh2->normals[i][2] = original_model[model](i, 5);
				}
				

				//ICP 配准

				Matrix4d p;
				trimesh::ICP_xform_type xform_type = trimesh::ICP_RIGID;

				trimesh::KDtree *kd2 = new trimesh::KDtree(mesh2->vertices);
				vector<float> weights1, weights2;
				double m[16];
				m[0] = 1;  m[1] = 0;  m[2] = 0;  m[3] = 0;
				m[4] = 0;  m[5] = 1;  m[6] = 0;  m[7] = 0;
				m[8] = 0;  m[9] = 0;  m[10] = 1; m[11] = 0;
				m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
				trimesh::xform xf1;
				trimesh::xform xf2;
				xf1 = m;
				xf2 = m;
				float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
					0.0f, 0, icpiterationcout, xform_type);
				//std::cout << xf2 << std::endl;
				p(0, 0) = xf2[0];
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

				icppose[model] = p.transpose() * icppose[model];
				MatrixXf r = transformPCPose(original_model[model], p.transpose());
				original_model[model] = r;
				
				delete mesh2;
				delete kd2;

			}
			delete mesh1;
			delete kd1;


		
		}
		
	}

	void PPF_PSO_Match::GlobaloptimizationICP(std::vector<MatrixXf> &original_model, double downsamplestep, double icpiterationcout, std::vector<Matrix4d> &icppose)
	{
	 /* 1.求体素并下采样
	    2.
	 */
		int modelsize = original_model.size();
		int totalpointnum=0;
		for (int model =0; model< modelsize; model++)
		{
			totalpointnum += original_model[model].rows();
		}
		icppose.resize(modelsize);

		MatrixXf  minmax(modelsize, 6);
		RowVector3f min, max;

		for (int model = 0; model < modelsize; model++)
		{
			minmax(model, 0) = original_model[model].col(0).minCoeff();
			minmax(model, 1) = original_model[model].col(1).minCoeff();
			minmax(model, 2) = original_model[model].col(2).minCoeff();
			minmax(model, 3) = original_model[model].col(0).maxCoeff();
			minmax(model, 4) = original_model[model].col(1).maxCoeff();
			minmax(model, 5) = original_model[model].col(2).maxCoeff();

			icppose[model].setIdentity();
		}
		for (int iter = 0; iter < icpiterationcout; iter++)
		{
			//cout<<"iter=" << iter << endl;
			
			clock_t tick1 = clock();
			for (int model = modelsize - 1; model > 0; model--)
			{
				//cout << model << endl;
				//下采样
				//体素化
				min(0) = minmax.col(0).minCoeff();
				min(1) = minmax.col(1).minCoeff();
				min(2) = minmax.col(2).minCoeff();

				max(0) = minmax.col(3).maxCoeff();
				max(1) = minmax.col(4).maxCoeff();
				max(2) = minmax.col(5).maxCoeff();
				double d = (min - max).norm();
				d = d * downsamplestep;
				//
				RowVector3i gridsum;
				for (int i = 0; i < 3; i++)
				{
					gridsum(i) = (max(i) - min(i)) / d + 1;
				}
				int totalsum = gridsum(0)*gridsum(1)*gridsum(2);

				std::vector<RowVector3f> point1, point2, point3;
				std::vector<RowVector3f> normal1, normal2, normal3;
				VectorXi pointsum1, pointsum2, pointsum3;
				point1.resize(totalsum);
				normal1.resize(totalsum);
				pointsum1.resize(totalsum);
				pointsum1.setZero();
				point3.resize(totalsum);
				normal3.resize(totalsum);
				pointsum3.resize(totalsum);
				pointsum3.setZero();
				point2.resize(totalsum);
				normal2.resize(totalsum);
				pointsum2.resize(totalsum);
				pointsum2.setZero();
				for (int i = 0; i < totalsum; i++)
				{
					point1[i].setZero();
					normal1[i].setZero();

					point2[i].setZero();
					normal2[i].setZero();

					point3[i].setZero();
					normal3[i].setZero();
				}

				//第一帧点云
				int n = original_model[0].rows();
				for (int i = 0; i < n; i++)
				{
					RowVector3f a = (original_model[0].row(i).head(3) - min)/d;
					int k = int(a(0))*gridsum(1)*gridsum(2) + int(a(1))*gridsum(2) + int(a(2));
					point1[k] += original_model[0].row(i).head(3);
					normal1[k] += original_model[0].row(i).tail(3);
					pointsum1(k) += 1;
				}

				/*n = original_model[model].rows();
				for (int i = 0; i < n; i++)
				{
					RowVector3f a = (original_model[model].row(i).head(3) - min)/ d;
					int k = int(a(0))*gridsum(1)*gridsum(2) + int(a(1))*gridsum(2) + int(a(2));
					point3[k] += original_model[model].row(i).head(3);
					normal3[k] += original_model[model].row(i).tail(3);
					pointsum3(k) += 1;
				}*/

				for (int mm = 1; mm < modelsize; mm++)
				{
					int n = original_model[mm].rows();
					if (model != mm)//其余点
					{
						for (int i = 0; i < n; i++)
						{
							RowVector3f a = (original_model[mm].row(i).head(3) - min) / d;
							int k = int(a(0))*gridsum(1)*gridsum(2) + int(a(1))*gridsum(2) + int(a(2));
							point2[k] += original_model[mm].row(i).head(3);
							normal2[k] += original_model[mm].row(i).tail(3);
							pointsum2(k) += 1;
						}
					}
				}

				/*for (int i = 0; i < gridsum(0); i++)
				{
					for (int j = 0; j < gridsum(1); j++)
					{
						for (int k = 0; k < gridsum(2); k++)
						{

							int kk = i * gridsum(1)*gridsum(2) + j * gridsum(2) + k;
							if (pointsum1(kk) >0)
							{

								for (int i1 = i-1; i1 < i+2; i1++)
								{
									if (i1 < 0 || i1 >= gridsum(0))
										continue;
									for (int j1 = i-1; j1 < j+2; j1++)
									{
										if (j1 < 0 || j1 >= gridsum(1))
											continue;

										for (int k1 = k-1; k1 < k+2; k1++)
										{
											if (k1 < 0 || k1 >= gridsum(2))
												continue;

											int kk = i1 * gridsum(1)*gridsum(2) + j1 * gridsum(2) + k1;
											pointsum2(kk)=0;
										}
									}
								}

							}
						}
					}
				}*/

				std::vector<RowVector3f> pointr1, pointr2;
				std::vector<RowVector3f> normalr1, normalr2;

				for (int i = 0; i < totalsum; i++)
				{
					if (pointsum1(i) > 0)
					{
						double d = normal1[i].norm();
						if (d > 0.0000001)
						{
							pointr1.push_back(point1[i] / pointsum1(i));
							normalr1.push_back(normal1[i] / d);
						}
							
					}
					else if (pointsum2(i) > 0)
					{
						double d = normal2[i].norm();
						if (d > 0.0000001)
						{
							pointr1.push_back(point2[i] / pointsum2(i));
							normalr1.push_back(normal2[i] / d);
						}
					}

					/*if (pointsum3(i) > 0)
					{
						double d = normal3[i].norm();
						if (d > 0.0000001)
						{
							pointr2.push_back(point3[i] / pointsum3(i));
							normalr2.push_back(normal3[i] / d);
						}
					}*/
				}


				trimesh::TriMesh *mesh1= new trimesh::TriMesh();;
				trimesh::TriMesh *mesh2=new trimesh::TriMesh();;

			

				mesh1->vertices.resize(pointr1.size());
				mesh1->normals.resize(pointr1.size());

				mesh2->vertices.resize(pointr2.size());
				mesh2->normals.resize(pointr2.size());
				
				n = pointr1.size();
				for (int i = 0; i < n; i++)
				{
					mesh1->vertices[i][0] = pointr1[i](0);
					mesh1->vertices[i][1] = pointr1[i](1);
					mesh1->vertices[i][2] = pointr1[i](2);

					mesh1->normals[i][0] = normalr1[i](0);
					mesh1->normals[i][1] = normalr1[i](1);
					mesh1->normals[i][2] = normalr1[i](2);
				}

			

				n = original_model[model].rows();
				mesh2->vertices.resize(n);
				mesh2->normals.resize(n);
				for (int i = 0; i < n; i++)
				{
					mesh2->vertices[i][0] = original_model[model](i,0);
					mesh2->vertices[i][1] = original_model[model](i, 1);
					mesh2->vertices[i][2] = original_model[model](i, 2);

					mesh2->normals[i][0] = original_model[model](i, 3);
					mesh2->normals[i][1] = original_model[model](i, 4);
					mesh2->normals[i][2] = original_model[model](i, 5);

				}
				/*n = pointr2.size();
				for (int i = 0; i < n; i++)
				{
					mesh2->vertices[i][0] = pointr2[i](0);
					mesh2->vertices[i][1] = pointr2[i](1);
					mesh2->vertices[i][2] = pointr2[i](2);

					mesh2->normals[i][0] = normalr2[i](0);
					mesh2->normals[i][1] = normalr2[i](1);
					mesh2->normals[i][2] = normalr2[i](2);
				}*/
				/*int kk = 0;for (int mm = 0; mm < modelsize; mm++)
				{
					int n = original_model[mm].rows();
					if (model != mm)//其余点
					{
						//if (model-1== mm || (model+1)% modelsize== mm)
						{
							for (int i = 0; i < n; i++)
						    {
								mesh1->vertices[kk][0] = original_model[mm](i, 0);
								mesh1->vertices[kk][1] = original_model[mm](i, 1);
								mesh1->vertices[kk][2] = original_model[mm](i, 2);

								mesh1->normals[kk][0] = original_model[mm](i, 3);
								mesh1->normals[kk][1] = original_model[mm](i, 4);
								mesh1->normals[kk][2] = original_model[mm](i, 5);

								kk++;
						    }
						}
						
					}
					else //移动点云
					{
						for (int i = 0; i < n; i++)
						{
							mesh2->vertices[i][0] = original_model[mm](i, 0);
							mesh2->vertices[i][1] = original_model[mm](i, 1);
							mesh2->vertices[i][2] = original_model[mm](i, 2);

							mesh2->normals[i][0] = original_model[mm](i, 3);
							mesh2->normals[i][1] = original_model[mm](i, 4);
							mesh2->normals[i][2] = original_model[mm](i, 5);
						}

					}
				}*/

				clock_t tick2 = clock();
				//ICP 配准

				Matrix4d p;
				trimesh::ICP_xform_type xform_type = trimesh::ICP_RIGID;
				

				trimesh::KDtree *kd1 = new  trimesh::KDtree(mesh1->vertices);
				trimesh::KDtree *kd2 = new trimesh::KDtree(mesh2->vertices);
				vector<float> weights1, weights2;
				double m[16];
				m[0] = 1;  m[1] = 0;  m[2] = 0;  m[3] = 0;
				m[4] = 0;  m[5] = 1;  m[6] = 0;  m[7] = 0;
				m[8] = 0;  m[9] = 0;  m[10] = 1; m[11] = 0;
				m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
				trimesh::xform xf1;
				trimesh::xform xf2;
				xf1 = m;
				xf2 = m;
				float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
					0.0f, 0,25, xform_type);
				//std::cout << xf2 << std::endl;
				p(0, 0) = xf2[0];
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

				icppose[model] = p.transpose() * icppose[model];
				MatrixXf r = transformPCPose(original_model[model], p.transpose());
				original_model[model] = r;

				minmax(model, 0) = original_model[model].col(0).minCoeff();
				minmax(model, 1) = original_model[model].col(1).minCoeff();
				minmax(model, 2) = original_model[model].col(2).minCoeff();
				minmax(model, 3) = original_model[model].col(0).maxCoeff();
				minmax(model, 4) = original_model[model].col(1).maxCoeff();
				minmax(model, 5) = original_model[model].col(2).maxCoeff();

				

				delete mesh1;
				delete mesh2;
				delete kd1;
				delete kd2;
				
			}
			clock_t tick3 = clock();
			//std::cout << std::endl << tick3 - tick1 << "  " << tick3 - tick1 << std::endl;
		}
	}

	

	void PPF_PSO_Match::match_PPF_MULTI_registration(MatrixXf& pc, std::vector<std::vector<SPose3D>>& resultsS, std::vector<int> &inst_count, const ppf_scenematching_parameter parameter)
	{
		//计算包围盒
		Vector2f xRange, yRange, zRange;
		computeBboxStd(pc, xRange, yRange, zRange); //验证
		
		//计算包围盒直径
		float dx = xRange[1] - xRange[0];
		float dy = yRange[1] - yRange[0];
		float dz = zRange[1] - zRange[0];
		float diameter = sqrt(dx * dx + dy * dy + dz * dz);
 
		//场景直径
		model_para.scene_diameter = diameter;
		
		//场景数量
		int model_size = model_para.model.size();
		model_para.scene.resize(model_size);

		//记录最大的采样点数和位置
		int pointnum = 0;
		int pointindx = 0;
	

		//保存场景的距离
		model_para.original_scene = pc;

		for (int model_id = 0; model_id < model_size; model_id++)
		{
			//计算场景下采样距离
			float SampleStepdistance = model_para.model[model_id].model_samplingdistance * parameter.relativeSceneDistance;
			//cout << "模型采样步长=" << model_id << "=" << model_para.model[model_id].model_samplingdistance << "  场景采样步长=" << SampleStepdistance << endl;
			model_para.scene[model_id].scene_samplingdistance = SampleStepdistance;
			
			VectorXi sumnum_scene;
			//场景下采样
			model_para.scene[model_id].downsample_scene = samplePCByQuantization_normal(pc, xRange, yRange, zRange, (float)SampleStepdistance, downsampleanglethreshold / 180 * 3.1415, downsamplelevel, sumnum_scene);//无加权点云下采样
			//场景查找树
			model_para.scene[model_id].scene_knn_tree = BuildKDTree(model_para.scene[model_id].downsample_scene.block(0, 0,
				model_para.scene[model_id].downsample_scene.rows(), 3));
			//记录最大的采样点数和位置
			if (model_para.scene[model_id].downsample_scene.rows() > pointnum)
			{
				pointnum = model_para.scene[model_id].downsample_scene.rows();
				pointindx = model_id;
			}


		}
		//用于计算重合度的场景点云
		model_para.combine_scene = model_para.scene[pointindx].downsample_scene;

		//用于计算重合度的查找树
		model_para.combine_scene_knn_tree = BuildKDTree(model_para.combine_scene.block(0, 0,
			model_para.combine_scene.rows(), 3));

	    //投票角度数量
		int numAngles = parameter.voteAnglenum;
	    //记录结果数组
		std::vector<std::vector<SPose3D>> resultsS3;
		resultsS3.resize(model_size);
		
		//对每个模型进行PPF投票
		for (int model_id = 0; model_id < model_size; model_id++)
		{
			//模型直径
			double model_diameter = model_para.model[model_id].model_diameter;
		
			//PPF离散化角度 跟模型的一直
			double ppf_a_step = angle_step_radians;

			//场景和模型的点数
			int scenerows = model_para.scene[model_id].downsample_scene.rows();
			int modelrows = model_para.model[model_id].downsample_model.rows();


			clock_t tick1 = clock();
			//采样间隔
			int stepsize = (int)(1.0 / parameter.samplingrate + 0.1);
			//进行投票
			for (int n = 0; n < scenerows; n += stepsize)
			{
				//n点的 坐标 和  法向
				Vector3d p1;
				p1 = model_para.scene[model_id].downsample_scene.block(n, 0, 1, 3).transpose().cast<double>();
				Vector3d n1;
				n1 = model_para.scene[model_id].downsample_scene.block(n, 3, 1, 3).transpose().cast<double>();

				//n点的 坐标 和  法向 变换到 原点的 平移和旋转
				Vector3d tsg;
				tsg.setZero();
				Matrix3d Rsg, RInv;
				Rsg.setZero();
				RInv.setZero();
				computeTransformRT(p1, n1, Rsg, tsg);

				//投票记录矩阵
				VectorXi accumulator = VectorXi::Zero(numAngles* modelrows);
				//用于是否进行第二次投票的判断矩阵
				VectorXi accumulatoryn = VectorXi::Ones(modelrows);
				//用于判断是否进行了同样的投票
				std::vector<unsigned long long> vote;
				//根据半径划分进行两次投票
				for (int radius = 0; radius < 2; radius++)
				{
					//点对形成的范围
					double radiusmin = line_minlenght;
					double radiusmax = 0.8;
					//第二次点对形成的范围
					if (radius == 1)
					{
						radiusmin = 0.8;
						radiusmax = 0.99;
					}

					{//遍历所有点与n点做成点对
						for (int jj = 0; jj < scenerows; jj++)
						{
							//第二点的位置和法向
							const Vector3d p2(model_para.scene[model_id].downsample_scene.block(jj, 0, 1, 3).transpose().cast<double>());
							const Vector3d n2(model_para.scene[model_id].downsample_scene.block(jj, 3, 1, 3).transpose().cast<double>());
							
							//求取场景角度
							double alpha_scene = computeAlphaRT(p2, Rsg, tsg);
							//记录投票位置
							int  alpha_sceneint = int((alpha_scene+ M_PI) / ppf_a_step+0.0000001);
							//计算特征值f
							Vector4d f;
							computePPFFeatures(p1, n1, p2, n2, f);//计算特征值f
												
							//是否是投票范围内的点			
							if (f[3] <= model_para.model[model_id].model_diameter* radiusmax
								&& f[3] > model_para.model[model_id].model_diameter * radiusmin)
							{
								//ppf特征索引				
								unsigned long long ppfindex = ((unsigned long long)(f[3] / model_para.model[model_id].model_ppf_distance_step)) * 1000000000 +
									((unsigned long long)(f[0] / angle_step_radians)) * 1000000 +
									((unsigned long long)(f[1] / angle_step_radians)) * 1000 +
									((unsigned long long)(f[2] / angle_step_radians));
								//记录投票的索引
								unsigned long long ppfvote = ppfindex * 1000 + alpha_sceneint;
								
								//判断是否已经进行了投票
								bool yn = true;
								for (int vi = vote.size() - 1; vi >= 0; vi--)
								{
									if (vote[vi] == ppfvote)
									{
										yn = false;
										break;
									}
								}
								if (yn)
								{//如果没有则进行投票
									//记录投票
									vote.push_back(ppfvote);

									//获得哈希值
									KeyType hashValue = hashPPF(f, angle_step_radians, model_para.model[model_id].model_ppf_distance_step);//计算索引哈希表索引
									//获得同一结点的哈希表指针
									hashnode_i* node = hashtableGetBucketHashed(model_para.model[model_id].hash_table, (hashValue));

									//对p2点实施变换
									Vector3d p2t = tsg + Rsg * p2.cast<double>();
									
									
									while (node)//遍历所有的结点上的模型PPF
									{

										//获得模型的ppf信息
										THash_MULTI * tData = (THash_MULTI *)node->data;
										int corrI = (int)tData->i;
										unsigned long long n = (unsigned long long)tData->ppfindex;


										//特征值一样进行投票
										if (accumulatoryn(corrI) && n == ppfindex)
										{
											//属性表位置
											int ppfInd = (int)tData->ppfInd;
											//模型ppf的特征角度
											double alpha_model = model_para.model[model_id].alpha[ppfInd];
											//旋转角度
											double alpha = alpha_model - alpha_scene;
											//投票的角度位置
											int alpha_index = (int)(numAngles*(alpha + 2 * M_PI) / (4 * M_PI));
											//计算索引
											uint accIndex = corrI * numAngles + alpha_index;
											//投票
											accumulator[accIndex]++;

										}
										node = node->next;

									}
								}
												
							}
										
							

						}

					}
					
					if (radius == 0)//第一次投票完成 选择第二次可以投票的位置
					{
						accumulatoryn.setZero();
						VectorXi accumulatortemp = accumulator;
						for (int num = 0; num < 10; num++)//选择第一次投票数最大10点模型点进行第二次投票
						{
							int refIndMax = 0;
							int maxVotes = 0;
							for (int kk = 0; kk < modelrows; kk++)
							{
								for (int j = 0; j < numAngles; j++)
								{
									const uint accInd = kk * numAngles + j;
									const uint accVal = accumulatortemp[accInd];
									if (accVal > maxVotes)
									{
										maxVotes = accVal;
										refIndMax = kk;
									}
								}
							}
							accumulatoryn(refIndMax) = 1;
							for (int j = 0; j < numAngles; j++)
							{
								const uint accInd = refIndMax * numAngles + j;
								accumulatortemp[accInd] = 0;
							}
						}
					}

				}

				//获取姿态
				// ///////////////////////////////////////////////////////////////////////////
				// 选择投票数量最大的两个点对应的位姿为所求姿态
				for (int num = 0; num < 2; num++)
				{
					int refIndMax = 0, alphaIndMax = 0;
					int maxVotes = 0;
					//查找最大位置及最大投票数
					for (int kk = 0; kk < modelrows; kk++)
					{
						for (int j = 0; j < numAngles; j++)
						{
							const uint accInd = kk * numAngles + j;
							const uint accVal = accumulator[accInd];
							if (accVal > maxVotes)
							{
								maxVotes = accVal;
								refIndMax = kk;
								alphaIndMax = j;
							}
						}
					}
					//////////////// 投票数必须大于设置的最小值 
					if (maxVotes < parameter.minpairnum)
					{
						break;
					}
					// ///////////////////////////////////////////////////////////////////////////	
				
					{ 
						//最大票数位置赋值为0
						const uint accInd = refIndMax * numAngles + alphaIndMax;
						accumulator[accInd] = 0;
						// invert Tsg : Luckily rotation is orthogonal: Inverse = Transpose.
						// We are not required to invert.
						Vector3d tInv, tmg;
						Matrix3d Rmg;
						//转置操作 变换旋转方向
						RInv = Rsg.transpose();
						tInv = -RInv * tsg;

						//合成场景变换位置
						Matrix4d TsgInv;
						rtToPose(RInv, tInv, TsgInv);

						//模型旋转变换
						Rmg = model_para.model[model_id].RMT[refIndMax];
						tmg = model_para.model[model_id].tMT[refIndMax];
						
						//合成模型变换位置
						Matrix4d Tmg;
						rtToPose(Rmg, tmg, Tmg);//拼接变换矩阵

						//将投票位置对应的角度转化为估计角度
						int alpha_index = alphaIndMax;
						double alpha = (alpha_index*(4 * M_PI)) / numAngles - 2 * M_PI;

						//求取alpha对应的变换矩阵:
						Matrix4d Talpha;
						Matrix3d R;
						Vector3d t;
						t.setZero();
						getUnitXRotation(alpha, R);
						rtToPose(R, t, Talpha);
						//求取模型到场景变换矩阵:
						SPose3D pose;
						pose.Pose = TsgInv * (Talpha * Tmg);

						{//记录最大投票数和位姿
							pose.numVotes = maxVotes;
							resultsS3[model_id].insert(resultsS3[model_id].begin(), pose);
						}
							
						

						if (resultsS3[model_id].size() > 6000)//如果数组超过6000删除掉最小2000  这个主要为了控制数组规模
						{
							std::sort(resultsS3[model_id].begin(), resultsS3[model_id].end(), pose3DPtrCompareS);
							resultsS3[model_id].erase(resultsS3[model_id].begin() + 4000, resultsS3[model_id].end());
						}

					}

				}
				// ///////////////////////////////////////////////////////////////////////////
			}
			clock_t tick2 = clock();
			//cout << "获得姿态数：" << resultsS3[0].size() << endl;
			if (resultsS3[0].size() < 1)//在没用获得位姿时主要设置一个单位变换，保证处理正常
			{
				SPose3D pose;
				pose.Pose.setIdentity();
				pose.numVotes = 5;
				resultsS3[0].push_back(pose);
				resultsS3[0].push_back(pose);
			}
			//对结果聚类
			std::vector<SPose3D> rs;
			std::vector<Matrix3d> rw;
			//模型数
			int rsize = resultsS3[model_id].size();
		     //按投票数排序
			std::sort(resultsS3[model_id].begin(), resultsS3[model_id].end(), pose3DPtrCompareS);
			//聚类的三个参考点
			MatrixXd ferencepoint(4, 3);
			ferencepoint =model_para.model[model_id].ferencepoint;
			//获得聚类的距离
			double dmax = model_para.model[model_id].model_diameter*parameter.Clusteringdistance;
			dmax = dmax * dmax;

			for (int i = 0; i < rsize; i++)
			{
				//获得变换后的参考点
				MatrixXd w=resultsS3[model_id][i].Pose.block(0, 0, 3, 4)*ferencepoint;
				//记录变换后的参考点
				resultsS3[model_id][i].referenceopint = w;
				//判断是否存在聚类半径内
				bool yn = true;
				int size = rw.size();
				for (int j = 0; j < size; j++)
				{
					double d=(w - rw[j]).squaredNorm();
					//cout << j << " " << d << " " << dmax << endl;;
					if (d < dmax)
					{
						//在半径内 累计投票数
						rs[j].numVotes = rs[j].numVotes+ resultsS3[model_id][i].numVotes;
						yn = false;
						break;
					}
				}
				if (yn)//不在加入新的点
				{
					rs.push_back(resultsS3[model_id][i]);
					rw.push_back(w);
				}
			}
			//清除位姿
			resultsS3[model_id].clear();
			//cout << "获得姿态数：" << resultsS3[model_id].size() << endl;
			rsize=rs.size();
		
			//赋值到位姿结果表中
			for (int i = 0; i < rsize; i++)
			{
				rs[i].overlap = 0;
				rs[i].oldoverlap = 0;
				resultsS3[model_id].push_back(rs[i]);
			}
			tick2 = clock();
			/*cout << endl << "voting time: "
				<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
				<< " sec" << endl << "Loading model..." << endl;*/
		}
		
		clock_t tick1 = clock();
		resultsS.resize(model_size);
		cout << "获得姿态数：" << resultsS3[0].size() << endl;
		std::vector<int> targetnum;
		std::vector<int> targetoverlapnum;

	    //初始重合度优化个数，目标数个数
		for (int model_id = 0; model_id < model_size; model_id++)//model_size
		{
			targetnum.push_back(inst_count[model_id]);
			targetoverlapnum.push_back(parameter.targetposenumber);//可以提高速度
		
		}

		for (int model_id = 0; model_id < model_size; model_id++)
		{
			int size = resultsS3[model_id].size();
			//按投票数排序
			std::sort(resultsS3[model_id].begin(), resultsS3[model_id].end(), pose3DPtrCompareS);

			volatile long targetoverlapcal = 0;
			for (int i = 0; i < size; i++)
			{
				//对投票数最大的前targetoverlapnum[model_id]个位姿计算重合度
				if (targetoverlapcal < targetoverlapnum[model_id])
				{
					//是否ICP
					if (parameter.ICP_enable == 1)
						ICP_MULTI_Optimizing_registration(model_id, resultsS3[model_id][i], parameter.ICP_nbIterations);
					//重合度计算
					estimatesampleangleoverlap_SPose3D(model_id, resultsS3[model_id][i], parameter.overlopthreshold, parameter.angleoverlopthreshold);
					//个数累计
					_InterlockedIncrement(&targetoverlapcal);
				}

			}
		
			size = resultsS3[model_id].size();
			if (size > 0)
			{
				//按重合度排序
				std::sort(resultsS3[model_id].begin(), resultsS3[model_id].end(), pose3DPtrCompareoverlapS);
				resultsS[model_id].push_back(resultsS3[model_id][0]);//选择第一个为配准最优结果
				
			}
		
    }

		

		clock_t tick2 = clock();
		/*cout << endl << "pose refinement time: "
			<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
			<< " sec" << endl << "Loading model..." << endl;*/

		


		for (int model_id = 0; model_id < model_size; model_id++)
		{

			for (int i = 0; i < resultsS[model_id].size(); i++)//计算6自由度参数
			{
				
				if (parameter.ICP_enable == 1)
					ICP_MULTI_Optimizing_registration(model_id, resultsS[model_id][i], 150);//ICP
				//平移和角度计算
				resultsS[model_id][i].translation = resultsS[model_id][i].Pose.block(0, 3, 3, 1);
				
				resultsS[model_id][i].aulerangle(0) = atan2(resultsS[model_id][i].Pose(2, 1), resultsS[model_id][i].Pose(2, 2)) / M_PI * 180;
				resultsS[model_id][i].aulerangle(1) = atan2(0 - resultsS[model_id][i].Pose(2, 0),
					sqrt(resultsS[model_id][i].Pose(2, 1) * resultsS[model_id][i].Pose(2, 1)
						+ resultsS[model_id][i].Pose(2, 2) * resultsS[model_id][i].Pose(2, 2))) / M_PI * 180;
				resultsS[model_id][i].aulerangle(2) = atan2(resultsS[model_id][i].Pose(1, 0), resultsS[model_id][i].Pose(0, 0)) / M_PI * 180;
			}
		}

	}
	void PPF_PSO_Match::findsimilararea(MatrixXf dr1, MatrixXf dr2, MatrixXf &r1, MatrixXf &r2, double diameter)
	{
		diameter = diameter * diameter;

		KDTree*  kdt1 = BuildKDTree(dr1.block(0, 0, dr1.rows(), 3));
		std::vector<std::vector<int>> indices;
		std::vector<std::vector<float>> dists;
		SearchKDTree(kdt1, dr2.block(0, 0, dr2.rows(), 3), indices, dists, 50);

		r2 = dr2;
		VectorXi index1(dr1.rows());
		index1.setZero();
		int trow2 = 0;
		int row2 = dr2.rows();
		for (int row = 0; row < row2; row++)
		{
			if (dists[row][0] < diameter)
			{
				r2.row(trow2) = dr2.row(row);
				trow2++;
			}
			for (int j = 0; j < 50; j++)
			{
				if (dists[row][j] < diameter)
				{
					index1(indices[row][j]) = 1;
				}
				else
				{
					break;
				}
			}
		}

		//std::cout << "trow2= " << trow2 << "  " << pointcloud[i2].rows() << "  " << index1.sum() << std::endl;
		r2.conservativeResize(trow2, 6);
		int trow1 = 0;
		r1 = dr1;
		int row1 = dr1.rows();
		for (int row = 0; row < row1; row++)
		{
			if (index1(row) == 1)
			{
				r1.row(trow1) = dr1.row(row);
				trow1++;
			}
		}
		//std::cout << "trow1= " << trow1 << "  " << pointcloud[i1].rows() << std::endl;
		r1.conservativeResize(trow1, 6);
	}
	void PPF_PSO_Match::PoseMatrixToAngle(Matrix4d pose, Vector3d & angle, Vector3d & translation)
	{
		translation = pose.block(0, 3, 3, 1);

		angle(0) = atan2(pose(2, 1), pose(2, 2)) / M_PI * 180;
		angle(1) = atan2(0 - pose(2, 0),sqrt(pose(2, 1) * pose(2, 1)+ pose(2, 2) * pose(2, 2))) / M_PI * 180;
		angle(2) = atan2(pose(1, 0), pose(0, 0)) / M_PI * 180;

	}
	void PPF_PSO_Match::computePPFFeatures(const Vector3d& p1, const Vector3d& n1,
		const Vector3d& p2, const Vector3d& n2,
		Vector4d& f)
	{
		Vector3d d(p2 - p1);
		//f[3] = cv::norm(d);
		f[3] = d.norm();
		if (f[3] <= EPS)
			return;
		d *= 1.0 / f[3];

		f[0] = TAngle3Normalized(n1, d);
		f[1] = TAngle3Normalized(n2, d);
		f[2] = TAngle3Normalized(n1, n2);
	}

	Matrix4d   PPF_PSO_Match::ICP_Optimizing(MatrixXf source, MatrixXf target, int max_iter)
	{
		trimesh::TriMesh *mesh1 = new trimesh::TriMesh();;
		trimesh::TriMesh *mesh2 = new trimesh::TriMesh();;

		//MatrixXf pct = transformPCPose(model_para.model[model_id].downsample_model, resultsS.Pose);//模型1
		//MatrixXf pct1 = model_para.scene[model_id].downsample_scene

		int modelpointnum1 = target.rows();
		int modelpointnum2 = source.rows();
		mesh1->vertices.resize(modelpointnum1);
		mesh1->normals.resize(modelpointnum1);

		mesh2->vertices.resize(modelpointnum2);
		mesh2->normals.resize(modelpointnum2);

		for (int i = 0; i < modelpointnum1; i++)
		{
			mesh1->vertices[i][0] = target(i, 0);
			mesh1->vertices[i][1] = target(i, 1);
			mesh1->vertices[i][2] = target(i, 2);

			mesh1->normals[i][0] = target(i, 3);
			mesh1->normals[i][1] = target(i, 4);
			mesh1->normals[i][2] = target(i, 5);
		}

		for (int i = 0; i < modelpointnum2; i++)
		{
			mesh2->vertices[i][0] = source(i, 0);
			mesh2->vertices[i][1] = source(i, 1);
			mesh2->vertices[i][2] = source(i, 2);

			mesh2->normals[i][0] = source(i, 3);
			mesh2->normals[i][1] = source(i, 4);
			mesh2->normals[i][2] = source(i, 5);
		}


		//ICP 配准

		clock_t tick1 = clock();
		Matrix4d p;
		trimesh::ICP_xform_type xform_type = trimesh::ICP_RIGID;


		trimesh::KDtree *kd1 = new  trimesh::KDtree(mesh1->vertices);
		trimesh::KDtree *kd2 = new trimesh::KDtree(mesh2->vertices);
		vector<float> weights1, weights2;
		double m[16];
		m[0] = 1;  m[1] = 0;  m[2] = 0;  m[3] = 0;
		m[4] = 0;  m[5] = 1;  m[6] = 0;  m[7] = 0;
		m[8] = 0;  m[9] = 0;  m[10] = 1; m[11] = 0;
		m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
		trimesh::xform xf1;
		trimesh::xform xf2;
		xf1 = m;
		xf2 = m;
		float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
			0.0f, 0, max_iter, xform_type);
		//
		p(0, 0) = xf2[0];
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
		//std::cout << p.transpose() << std::endl;
	
		delete mesh1;
		delete mesh2;
		delete kd1;
		delete kd2;

		return p.transpose();
	}
	void  PPF_PSO_Match::ICP_MULTI_Optimizing_registration(int model_id, SPose3D & resultsS, int max_iter)
	{


		MatrixXf pct = transformPCPose(model_para.model[model_id].downsample_model, resultsS.Pose);//模型1


		MatrixXf  r1, r2;
		r1 = model_para.scene[model_id].downsample_scene;
		r2 = pct;
		//std::cout << r2.rows() << " " << r1.rows() << std::endl;

		findsimilararea(model_para.scene[model_id].downsample_scene, pct, r1, r2, model_para.model[model_id].model_samplingdistance*4);

		//std::cout << r2.rows() << " " << r1.rows() << std::endl;
		//Matrix4d pose1=detectorpso.ICP_Optimizing(pointcloud[i2], pointcloud[i1], 150);
		Matrix4d pose1;
		pose1.setIdentity();
		if(r2.rows()>50 &&  r1.rows()>50 )
			pose1 = ICP_Optimizing(r2, r1, max_iter);
		Vector3d   angle, translation;
		PoseMatrixToAngle(pose1, angle, translation);
		//std::cout  << angle.transpose() << " " << translation.norm() << " " << model_para.model[model_id].model_diameter*0.5 << std::endl << pose1 << std::endl;
		if (fabs(angle(0)) < 15
			&& fabs(angle(1)) < 15
			&& fabs(angle(2)) < 15
			&& translation.norm() < model_para.model[model_id].model_diameter*0.5)
		{
			resultsS.Pose = pose1 * resultsS.Pose;
		}

	}


	void  PPF_PSO_Match::ICP_MULTI_Optimizing_registration(int model_id, SPose3D & resultsS, const ppf_scenematching_parameter parameter)
	{
	
		double thresholdlen = 0.1*model_para.model[model_id].model_diameter;
		thresholdlen = thresholdlen * thresholdlen;

		int nbIterations = parameter.ICP_nbIterations;
		int nbIterationsIn = 5;
		double mu = 8;
		int nbIterShrink = 3;
		double p = 0.5;
		IcpMethod method = pointToPlane;
		bool verbose = false;
		size_t kNormals = 8;
		double th_angle = cos(3.1415926*45.0/ 180.0);
		for (int i = 0; i < nbIterations; i++)
		{
			MatrixXf pct = transformPCPose(model_para.model[model_id].downsample_model, resultsS.Pose);
			int n = 0;
			int rows = pct.rows();
			double aa = 0;
			for (int j = 0; j < rows; j++)
			{
				if (pct(j, 5) < aa)
				{
					pct.row(n) = pct.row(j);
					n++;
				}
			}
			pct.conservativeResize(n, 6);

			std::vector<std::vector<int>> indices;
			std::vector<std::vector<float>> dists;
			SearchKDTree(model_para.scene[model_id].scene_knn_tree, pct.block(0, 0, n, 3), indices, dists, 5);

			MatrixXf movepoint(n, 6);
			MatrixXf targetpoint(n, 6);
	
			int num = 0;
			for (int i = 0; i < n; i++)
			{

				RowVector3f r1 = pct.row(i).tail(3);
				for (int j = 0; j <5 ; j++)
				{
					RowVector3f r2 = model_para.scene[model_id].downsample_scene.row(indices[i][j]).tail(3);
					double a = r1.dot(r2);
					if (dists[i][j] < thresholdlen && a>th_angle)
					{
						movepoint.row(num) = pct.row(i);
						targetpoint.row(num) = model_para.scene[model_id].downsample_scene.row(indices[i][j]);
						num++;
						break;
					}
				}
				
			}
			if (num < 10)
			{
				return;
			}
			//cout << num << endl;
			movepoint.conservativeResize(num, 6);
			targetpoint.conservativeResize(num, 6);

			Matrix4d Pose;
			{
				IcpOptimizer myIcpOptimizer(movepoint.cast<double>(), targetpoint.cast<double>(), kNormals, nbIterations, nbIterationsIn, mu, nbIterShrink, p, method, verbose, 2);
				int hasIcpFailed = myIcpOptimizer.performSparceICP_single();
				if (hasIcpFailed)
				{
					cerr << "Failed to load the point clouds. Check the paths." << endl;
				}
				RigidTransfo resultingTransfo = myIcpOptimizer.getComputedTransfo();
				//cout << "Computed Rotation : " << endl << resultingTransfo.first << endl << "Computed Translation : " << endl << resultingTransfo.second << endl;
				MatrixXd P(3, 4);
				P << resultingTransfo.first, resultingTransfo.second;//横行向合并
				Pose << P,
					RowVector4d(0, 0, 0, 1);
			}
			resultsS.Pose = Pose * resultsS.Pose;
			//cout << Pose << endl;
			//MatrixXf r = transformPCPose(original_model[model], Pose);
		}
	}

	
	void PPF_PSO_Match::estimatesampleangleoverlap_SPose3D(int model_id, SPose3D &results, double threshold, double anglethreshold)
	{
		
		double thresholdlen = threshold * model_para.model[model_id].model_diameter;
		thresholdlen = thresholdlen * thresholdlen;

		double angle = -1.01;
		if (anglethreshold < 180)
			angle = cos(anglethreshold / 180 * M_PI);

		int pointnum = 0;
	

		int  rows = model_para.model[model_id].downsample_model.rows();
		MatrixXf pct = transformPCPose(model_para.model[model_id].downsample_model, results.Pose);
		
		rows= pct.rows();
		std::vector<std::vector<int>> indices;
		std::vector<std::vector<float>> dists;
		SearchKDTree(model_para.combine_scene_knn_tree, pct.block(0, 0, pct.rows(), 3), indices, dists, 15);
		
		for (int j = 0; j < rows; j++)
		{
			
			{
				int x1 = indices[j].size();
				int x2= dists[j].size();
				//cout << indices[j][0] << endl;
				//cout<< j << "sdd " << x1 << endl;
				Vector3f n1 = pct.row(j).tail(3).transpose();
				Vector3f n2 = model_para.combine_scene.row(indices[j][0]).tail(3).transpose();
				double a = n1.dot(n2);
				{
					if (dists[j][0] < thresholdlen)
					{
						
						if (a > angle)
						{
							pointnum++;// sumnum_model(j);
						}


					}

				}

			}


		}

		results.overlap = 1.0* pointnum / model_para.model[model_id].downsample_model.rows(); //sumnum_model.sum(); //;pointnum*pointnum*100.1 /
		results.oldoverlap = 1.0;// 1.0 *pointnum4 / pointnum3;//配准为1.0 检测为 1.0 *pointnum4 / pointnum3
		results.overnum = pointnum;
	


		//cout << results.overlap << endl;
	}


}
}