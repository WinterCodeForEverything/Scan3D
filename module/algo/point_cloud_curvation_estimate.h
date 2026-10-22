#ifndef SCAN3D_ALGO_POINT_CLOUD_CURVATION_ESTIMATE_H
#define SCAN3D_ALGO_POINT_CLOUD_CURVATION_ESTIMATE_H

#include<vector>
#include<3rd_party/kd_tree/ANN/ANN.h>

namespace scan3d
{
	class PointCloud;
	typedef struct {
		double x, y, z;
	}Point_t, Vector_t;

	typedef struct {
		int		  vNum;
		Point_t   *V;
		Vector_t   *Vn;
		Vector_t   *Dir1;
		Vector_t   *Dir2;
		double     *K1;
		double     *K2;
		double     K1Box[2], K2Box[2];
	}PCDAnaOBJ_t;
	class PointCloudCurvationEstimate
	{
	public :
		PointCloudCurvationEstimate();

		/// Estimates the point cloud curvation.
		/// @param numNeighbor: the number of neighboring points to estimate curvation.
		std::vector<std::vector<double>> curvation_estimate( PointCloud* cloud, int numNeighbor );
	private:
		int InitTObjFile(PCDAnaOBJ_t *pOBJ);
		int FreeTObjFile(PCDAnaOBJ_t *pOBJ);
		int ReadPointCloudPtsxyz(PointCloud* cloud, PCDAnaOBJ_t *pPts);

		//-----------------main algorithm -----------------------
		int CreateGlobKdTree(void);//int neibNum
		int FreeGlobKdTree(void);
		int NormalVectorCalculatingWithQFit(PCDAnaOBJ_t *pPtCdd, int numNeighbor);
		int curvatureCalculatingWithCAN(PCDAnaOBJ_t *pPtCdd, int numNeighbor);
	
	private:
		char buf[100];
		PCDAnaOBJ_t *pScanD;
		ANN::ANNkd_tree   *GkdTree;
		ANN::ANNpointArray dataPts;

	};
}



#endif  //SCAN3D_ALGO_POINT_CLOUD_CURVATION_ESTIMATE_H