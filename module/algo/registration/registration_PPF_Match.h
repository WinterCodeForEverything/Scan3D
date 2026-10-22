
#ifndef __PPF_MATCH_3D_H__
#define __PPF_MATCH_3D_H__


#include <vector>
#include "registration_ObjectOpse.h"
#include "registration_t_hash_int.hpp"
#include "3rd_party/eigen/Eigen/Dense"
#include "registration_stdafx.h"
#include "registration_ppf_helpers.hpp"
#include "3rd_party/kd_tree/FLANN/flann.hpp"
#include<omp.h>
#include <string>

#include "3rd_party/kd_tree/nanoflann/nanoflann.hpp"

using namespace Eigen;

static omp_lock_t lock;
namespace play3d {
	
	namespace ppf_match
	{
		using namespace nanoflann;
		typedef KDTreeEigenMatrixAdaptor<Matrix<float, Dynamic, 3> > my_kd_tree_t;
		typedef KDTreeEigenMatrixAdaptor< Matrix<double, Dynamic, 33> > feature_kd_tree_t;
		/**
		* @brief Struct, holding a node in the hashtable
			*/
		typedef struct THash
		{
			int id;
			int i, ppfInd;
		} THash;

		typedef struct THash_MULTI
		{
			int i;
			int ppfInd;
			unsigned long long ppfindex;
		} THash_MULTI;

	


		typedef struct PPF_PSO_MODEL
		{
			std::string path;
			MatrixXf  original_model;
			MatrixXf  score_model;
			MatrixXf  downsample_model;
			VectorXi edgeid;
			MatrixXi face;
			//MatrixXf ppf_coordinate;
			std::vector<int> ppf;
			std::vector<Matrix3d> ppf_coordinate;
			std::vector<Matrix3d> ppf_coordinate2;


			//MatrixXf ppf_coordinate;
			//std::vector<int> ref_j;
			std::vector<Matrix3d> RMT;
			std::vector<Vector3d> tMT;
			std::vector<double>  alpha;

			std::vector<SPlane> model_Plane;
			Vector3d model_center;
			Vector3d model_referencepoint1;
			Vector3d model_referencepoint2;

			


			MatrixXd ferencepoint;

			double model_diameter;

			hashtable_int* hash_table;
			THash_MULTI* hash_nodes;

			double model_samplingdistance;
			double model_ppf_distance_step;

		};
		typedef struct PPF_PSO_SCENE
		{
			MatrixXf  downsample_scene;
			double scene_samplingdistance;
			KDTree* scene_knn_tree;
			Vector3d mincorner;
			int Gridnum_x;
			int Gridnum_y;
			int Gridnum_z;
			double gridnum_dl;
		};
		


		typedef struct PPF_MODEL_MULTI
		{
			std::vector<PPF_PSO_MODEL> model;
			std::vector<PPF_PSO_SCENE> scene;
			MatrixXf  original_scene;
			MatrixXf  scene_edge;
			KDTree*   scene_edge_knn_tree;
			MatrixXf  combine_scene;
			double scene_diameter;
			KDTree* combine_scene_knn_tree;

		};



		

		typedef struct ppf_model_parameter
		{
			double relativeSamplingStep;
			double relativeDistanceStep;
			double numAngles;
			int numplanes;
			double threshold_Angles;
			double thresholdlencur;
			std::string method;
			double pointtoplanethreshold;
			double line_minlenght;
			double downsampleanglethreshold;
			int downsamplelevel;
		};
		typedef struct ppf_scenematching_parameter
		{
			double relativeSceneDistance;
			int minpairnum;
			int voteAnglenum;
			double  samplingrate;
			double targetposenumber;
			int results_size;

			int PSO_particlenum;
			int PSO_groupnum;
			int PSO_iteration;
			double PSO_randthreshold;
			double PSO_velweight;
			double PSO_c1;
			double PSO_c2;

			double minoverlop;
			double overlopthreshold;
			double angleoverlopthreshold;
			double Secondevaluationcriterion;
			double ICP_enable;
			double ICP_nbIterations;
			int background;

			double maxvote;
			double minvote;
			double  Clusteringdistance;
			double Overlapdistance;

		};
	
		class  PPF_PSO_Match
		{
		public:
			PPF_PSO_Match();
			PPF_PSO_Match(ppf_model_parameter model_para);
			~PPF_PSO_Match();
			void trainModel_PPF(const std::vector<MatrixXf> &original_model);
			void match_PPF_MULTI_registration(MatrixXf& pc, std::vector<std::vector<SPose3D>>& resultsS, std::vector<int> &inst_count, const ppf_scenematching_parameter parameter);
			/// 功能：实现连续配准的全局优化
		    /// @参数 original_model: 所要优化的点云 
		    /// @参数 downsamplestep: 下采样体素步长，模型包围盒的相对距离
		    /// @参数 icpiterationcout: 迭代次数
			/// @参数 icppose: 优化结果，4*4的变换矩阵列表
			void GlobaloptimizationICP(std::vector<MatrixXf> &original_model, double downsamplestep, double icpiterationcout, std::vector<Matrix4d> &icppose);
			void GlobaloptimizationICP_fine(std::vector<MatrixXf> &original_model, double downsamplestep, double icpiterationcout, std::vector<Matrix4d> &icppose);
			Matrix4d  ICP_Optimizing(MatrixXf source, MatrixXf target, int max_iter);
			void PoseMatrixToAngle(Matrix4d pose, Vector3d & angle, Vector3d & translation);
			void findsimilararea(MatrixXf dr1, MatrixXf dr2, MatrixXf &r1, MatrixXf &r2,double diameter);
		private:
			
			void  estimatesampleangleoverlap_SPose3D(int model_id, SPose3D &results, double threshold, double anglethreshold);
			void  ICP_MULTI_Optimizing_registration(int model_id, SPose3D & resultsS, const ppf_scenematching_parameter parameter);
			void  ICP_MULTI_Optimizing_registration(int model_id, SPose3D & resultsS, int max_iter);
			static void  computePPFFeatures(const Vector3d& p1, const Vector3d& n1,const Vector3d& p2, const Vector3d& n2,Vector4d& f);
		public:
			PPF_MODEL_MULTI model_para;
		private:
			double sampling_step_relative;//采样步长
			double distance_step_relative;//ppf离散化步长 相对步长
			double angle_step_relative;//ppf离散角度数
			double angle_step_radians;//ppf角度李赛步数
			double angle_step ;//离散步数
			std::vector<int> modelplane;
			double threshold_Angles;//角度阈值
			double line_minlenght;//最小距离阈值
			double pointtoplanethreshold;//点到平面距离
			double downsamplelevel;//下采样层数
			double downsampleanglethreshold ;//下采样角度
		};

		
		//! @}

	} // namespace ppf_match_3d

}
#endif
