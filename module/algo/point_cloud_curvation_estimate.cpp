#include<stdio.h>
#include<math.h>
#include<fstream>
#include <vector>
#include<time.h>

#include <module/fileio/resources.h>
#include <module/util/logging.h>
#include <module/core/point_cloud.h>
#include <module/core/types.h>


#include <module/algo/svdSolveEquation.h>
#include <module/algo/point_cloud_curvation_estimate.h>
#include <module/util/stop_watch.h>

//using namespace std;
namespace scan3d
{
	//-----------------------------------------
	PointCloudCurvationEstimate::PointCloudCurvationEstimate()
	{
		pScanD = new PCDAnaOBJ_t;
	}

	//--------------------------------------------------------------

	std::vector<std::vector<double>> PointCloudCurvationEstimate::curvation_estimate(PointCloud* cloud, int numNeighbor)//cloud represents a point cloud£¬numNeighbor indicates the number of neighbors
	{
		LOG(INFO) << "start estimate curvation.";
		StopWatch w;
		InitTObjFile(pScanD);
		int opend = ReadPointCloudPtsxyz(cloud, pScanD);
		CreateGlobKdTree();
		pScanD->Vn = new Vector_t[pScanD->vNum];
		NormalVectorCalculatingWithQFit(pScanD, numNeighbor);
		pScanD->Dir1 = new Vector_t[pScanD->vNum];
		pScanD->Dir2 = new Vector_t[pScanD->vNum];
		pScanD->K1 = new double[pScanD->vNum];
		pScanD->K2 = new double[pScanD->vNum];
		curvatureCalculatingWithCAN(pScanD, numNeighbor);
		std::vector<std::vector<double>> result(pScanD->vNum);
		for (int i = 0; i < pScanD->vNum; i++)
		{
			result[i] = std::vector<double>{ pScanD->K1[i], pScanD->K2[i] };
		}
		FreeTObjFile(pScanD);
		LOG(INFO) << "estimate curvation finished, total time:" << w.time_string();
		return result;
	}

	int PointCloudCurvationEstimate::InitTObjFile(PCDAnaOBJ_t *pOBJ)
	{
		pOBJ->vNum = 0;
		pOBJ->V = NULL;//The coordinates of all nodes
		pOBJ->Vn = NULL;
		pOBJ->Dir1 = NULL;
		pOBJ->Dir2 = NULL;
		pOBJ->K1 = NULL;
		pOBJ->K2 = NULL;
		return 1;
	}

	int PointCloudCurvationEstimate::FreeTObjFile(PCDAnaOBJ_t *pOBJ)
	{
		if (pOBJ->V)
			delete[]pOBJ->V;//The coordinates of all nodes
		if (pOBJ->Vn)
			delete[]pOBJ->Vn;
		if (pOBJ->Dir1)
			delete[]pOBJ->Dir1;
		if (pOBJ->Dir2)
			delete[]pOBJ->Dir2;
		if (pOBJ->K1)
			delete[]pOBJ->K1;
		if (pOBJ->K2)
			delete[]pOBJ->K2;
		return 1;

	}

	int PointCloudCurvationEstimate::ReadPointCloudPtsxyz(PointCloud* cloud, PCDAnaOBJ_t* pPts)
	{
		std::vector<vec3>& points = cloud->points();
		float x, y, z;
		pPts->vNum = points.size();
		pPts->V = new Point_t[pPts->vNum];
		for (int i = 0; i < points.size(); i++)
		{
			pPts->V[i].x = double(points[i][0]);
			pPts->V[i].y = double(points[i][1]);
			pPts->V[i].z = double(points[i][2]);
		}
		return pPts->vNum;
	}

	//-----------------basic algorithms -----------------------
	int PointCloudCurvationEstimate::CreateGlobKdTree(void) //int neibNum
	{
		//ANNpointArray dataPts;
		dataPts = ANN::annAllocPts(pScanD->vNum, 3);
		for (int i = 0; i < pScanD->vNum; i++) {
			dataPts[i][0] = pScanD->V[i].x;
			dataPts[i][1] = pScanD->V[i].y;
			dataPts[i][2] = pScanD->V[i].z;
		}
		GkdTree = NULL;
		GkdTree = new ANN::ANNkd_tree(dataPts, pScanD->vNum, 3);
		if (GkdTree)
			return 1;
		else
			return 0;
	}

	int PointCloudCurvationEstimate::FreeGlobKdTree(void) //int neibNum
	{
		//ANNpointArray dataPts;
		delete GkdTree;
		GkdTree = NULL;
		return 1;
	}

	int PointCloudCurvationEstimate::NormalVectorCalculatingWithQFit(PCDAnaOBJ_t *pPtCdd, int numNeighbor)
	{
		//---------------czlPMA2006 ----------  

		int i, j;
		int numberPts = pPtCdd->vNum;
		Point_t *nrmPts = pPtCdd->Vn;
		Point_t *Pts = pPtCdd->V;

		Point_t *nrm0 = new Point_t[numberPts];  //Temporary initial normal vector
		Point_t *PtNeighbor = new Point_t[numNeighbor];//Used to store neighbor data
		Point_t *nrmNeighbor = new Point_t[numNeighbor];//Temporarily store neighbor coordinates
		int   *qArraySN = new int[numNeighbor];//Temporarily store the number of the neighbors in the array

		int rowOfM = numNeighbor; // Number of rows in M
		float **M = new float*[numNeighbor + 1];  //svdcmp(M,  rowOfM, 3, ww, vv);
		for (i = 0; i < numNeighbor + 1; i++)
			M[i] = new float[4];
		float *ww = new float[4];
		float **vv = new float*[numNeighbor + 1];
		for (i = 0; i < numNeighbor + 1; i++)
			vv[i] = new float[4];

		int k;
		float minW;

		Point_t p;
		float *nrmTp = new float[3];

		float vectorLen = 0, delta_norm = (numNeighbor - 1) / 2 * (4.0e-1), sumWeight = 0;
		float *weight = new float[numNeighbor];

		//--------  step 1 ------------------------------

		ANN::ANNpoint      queryPt;
		ANN::ANNidxArray   nnIdx;
		ANN::ANNdistArray  dists;
		double epsKdtree = 0;
		queryPt = ANN::annAllocPt(3);

		nnIdx = new ANN::ANNidx[numNeighbor + 1];
		dists = new ANN::ANNdist[numNeighbor + 1];

		for (i = 0; i < numberPts; i++) { 
			nrm0[i].x = 0;  nrm0[i].y = 0;  nrm0[i].z = 0;
			//Find numNeighbor nearest neighbors, calculate PQi, and convert it to a unit vector
			p = Pts[i];
			nrmTp[0] = 0; nrmTp[1] = 0; nrmTp[2] = 0; sumWeight = 0;
			queryPt[0] = Pts[i].x;
			queryPt[1] = Pts[i].y;
			queryPt[2] = Pts[i].z;
			GkdTree->annkSearch(queryPt, numNeighbor + 1, nnIdx, dists, epsKdtree);
			for (j = 0; j < numNeighbor; j++)
				qArraySN[j] = nnIdx[j + 1];

			for (j = 0; j < numNeighbor; j++) {
				PtNeighbor[j].x = Pts[qArraySN[j]].x - p.x;
				PtNeighbor[j].y = Pts[qArraySN[j]].y - p.y;
				PtNeighbor[j].z = Pts[qArraySN[j]].z - p.z;
				vectorLen = sqrtf(PtNeighbor[j].x*PtNeighbor[j].x + PtNeighbor[j].y*PtNeighbor[j].y + PtNeighbor[j].z*PtNeighbor[j].z);
				weight[j] = vectorLen / delta_norm;
				if (weight[j] >= 1) {
					weight[j] = 0;
				}
				else {
					weight[j] = (1 - weight[j])*(1 - weight[j])*(1 - weight[j])*(1 - weight[j])*(1 + 4 * weight[j]);
					sumWeight = sumWeight + weight[j];
					nrmTp[0] = nrmTp[0] + Pts[qArraySN[j]].x*weight[j];
					nrmTp[1] = nrmTp[1] + Pts[qArraySN[j]].y*weight[j];
					nrmTp[2] = nrmTp[2] + Pts[qArraySN[j]].z*weight[j];
				}
			}
			nrmTp[0] = nrmTp[0] / sumWeight;
			nrmTp[1] = nrmTp[1] / sumWeight;
			nrmTp[2] = nrmTp[2] / sumWeight;

			rowOfM = 0;
			for (j = 0; j < numNeighbor; j++) {
				if (weight[j] > 0) {
					rowOfM++;
					M[rowOfM][1] = (Pts[qArraySN[j]].x - nrmTp[0]) * weight[j];
					M[rowOfM][2] = (Pts[qArraySN[j]].y - nrmTp[1]) * weight[j];
					M[rowOfM][3] = (Pts[qArraySN[j]].z - nrmTp[2]) * weight[j];
				}
			}
			for (j = 1; j <= 3; j++) {
				vv[j][1] = 1.0; vv[j][2] = 1.0; vv[j][3] = 1.0;
				ww[j] = 0.0;
			}
			svdcmp(M, rowOfM, 3, ww, vv);

			if (ww[1] <= ww[2]) {
				ww[0] = ww[1]; minW = ww[2]; k = 1;
			}
			else {
				ww[0] = ww[2]; minW = ww[1]; k = 2;
			}
			if (ww[3] <= ww[0]) {
				minW = ww[0]; ww[0] = ww[3]; k = 3;
			}
			else if (ww[3] <= minW)
				minW = ww[3];


			if (minW > 1e-16) {
				nrm0[i].x = -vv[1][k];
				nrm0[i].y = -vv[2][k];
				nrm0[i].z = -vv[3][k];
			}
		}

		//--------  step 2 normalvoting ------------------------
		float *weight2 = new float[numNeighbor];
		float *yy = new float[numNeighbor];
		float *alpha = new float[numNeighbor];
		float *opi = new float[numNeighbor];
		Point_t *nrv = new Point_t[numberPts];  //
		int *idx = new int[numNeighbor];
		int idx2;
		float eps = 1.0e-12;

		for (i = 0; i < numberPts; i++) { //for(i=0;i<numberPts;i++)
			nrmPts[i].x = nrm0[i].x;
			nrmPts[i].y = nrm0[i].y;
			nrmPts[i].z = nrm0[i].z;

			p = Pts[i];
			nrmTp[0] = 0;
			nrmTp[1] = 0;
			nrmTp[2] = 0;
			sumWeight = 0;
			idx2 = 0;

			queryPt[0] = Pts[i].x;
			queryPt[1] = Pts[i].y;
			queryPt[2] = Pts[i].z;
			GkdTree->annkSearch(queryPt, numNeighbor + 1, nnIdx, dists, epsKdtree);
			for (j = 0; j < numNeighbor; j++)
				qArraySN[j] = nnIdx[j + 1];

			for (j = 0; j < numNeighbor; j++) {
				PtNeighbor[j].x = Pts[qArraySN[j]].x - p.x;//dif[][]
				PtNeighbor[j].y = Pts[qArraySN[j]].y - p.y;
				PtNeighbor[j].z = Pts[qArraySN[j]].z - p.z;

				nrmNeighbor[j].x = nrm0[qArraySN[j]].x;//nr[][]
				nrmNeighbor[j].y = nrm0[qArraySN[j]].y;
				nrmNeighbor[j].z = nrm0[qArraySN[j]].z;

				yy[j] = PtNeighbor[j].x*nrmNeighbor[j].x + PtNeighbor[j].y*nrmNeighbor[j].y + PtNeighbor[j].z*nrmNeighbor[j].z;

				vectorLen = PtNeighbor[j].x*PtNeighbor[j].x + PtNeighbor[j].y*PtNeighbor[j].y + PtNeighbor[j].z*PtNeighbor[j].z;
				alpha[j] = asin(fabs(yy[j]) / sqrtf(vectorLen + eps));

				weight[j] = sqrtf(vectorLen) / delta_norm;
				if (weight[j] >= 1)
					weight[j] = 1;
				weight[j] = (1 - weight[j])*(1 - weight[j])*(1 - weight[j])*(1 - weight[j])*(1 + 4 * weight[j]);

				if (alpha[j] > 0 && alpha[j] <= M_PI / 4) {
					idx[j] = 1;
					nrv[idx2].x = vectorLen / yy[j] / 2 * nrmNeighbor[j].x - PtNeighbor[j].x;
					nrv[idx2].y = vectorLen / yy[j] / 2 * nrmNeighbor[j].y - PtNeighbor[j].y;
					nrv[idx2].z = vectorLen / yy[j] / 2 * nrmNeighbor[j].z - PtNeighbor[j].z;

					vectorLen = sqrtf(nrv[idx2].x*nrv[idx2].x + nrv[idx2].y*nrv[idx2].y + nrv[idx2].z*nrv[idx2].z);
					nrv[idx2].x = nrv[idx2].x / vectorLen;
					nrv[idx2].y = nrv[idx2].y / vectorLen;
					nrv[idx2].z = nrv[idx2].z / vectorLen;

					weight2[idx2] = weight[j];

					idx2++;
				}
				else
					idx[j] = 0;

			}
			for (j = 0; j < numNeighbor; j++) {
				if (alpha[j] < eps) {
					nrv[idx2].x = nrmNeighbor[j].x;
					nrv[idx2].y = nrmNeighbor[j].y;
					nrv[idx2].z = nrmNeighbor[j].z;
					weight2[idx2] = weight[j];
					idx2++;
					if (idx2 == numNeighbor)
						break;
				}
			}

			rowOfM = 0;
			for (j = 0; j < numNeighbor; j++) {
				if (weight2[j] > 0) {
					rowOfM++;
					M[rowOfM][1] = nrv[j].x * weight2[j];
					M[rowOfM][2] = nrv[j].y * weight2[j];
					M[rowOfM][3] = nrv[j].z * weight2[j];
				}
			}
			for (j = 1; j <= 3; j++) {
				vv[j][1] = 1.0; vv[j][2] = 1.0; vv[j][3] = 1.0;
				ww[j] = 0.0;
			}
			svdcmp(M, rowOfM, 3, ww, vv);


			if (ww[1] >= ww[2]) {
				ww[0] = ww[1]; k = 1;
			}
			else {
				ww[0] = ww[2]; k = 2;
			}
			if (ww[3] > ww[0]) {
				ww[0] = ww[3]; k = 3;
			}
		}

		delete[]PtNeighbor;
		delete[]nrmNeighbor;
		delete[]qArraySN;
		delete[]nrmTp;
		for (i = 0; i < numNeighbor; i++)
			delete M[i];
		delete[]M;
		delete[]ww;
		for (i = 0; i < numNeighbor + 1; i++)
			delete vv[i];
		delete[]vv;
		delete[]weight;

		delete[]weight2;
		delete[]yy;
		delete[]alpha;
		delete[]opi;
		delete[]nrv;
		delete[]idx;

		delete[]nnIdx;
		delete[] dists;
		return 1;
	}

	int PointCloudCurvationEstimate::curvatureCalculatingWithCAN(PCDAnaOBJ_t *pPtCdd, int numNeighbor)
	{
		Point_t *Pts = pPtCdd->V;
		int     numberPts = pPtCdd->vNum;
		Point_t *nrmPts = pPtCdd->Vn;
		double *K1 = pPtCdd->K1;
		double *K2 = pPtCdd->K2;
		Point_t *PriDirection1 = pPtCdd->Dir1;
		Point_t *PriDirection2 = pPtCdd->Dir2;
		int i, j;
		Point_t *PtNeighbor = new Point_t[numNeighbor];
		Point_t *nrmNeighbor = new Point_t[numNeighbor];
		int   *qArraySN = new int[numNeighbor];

		float **M = new float*[3];
		for (i = 0; i < 3; i++)
			M[i] = new float[3];
		float **invM = new float*[3];
		for (i = 0; i < 3; i++)
			invM[i] = new float[3];
		float *b = new float[3];
		float detM = 0;

		float A, B, C;//Weingarten matrix [A, B; B, C]
		float *ox = new float[3];
		float *oy = new float[3];
		float *jubuhua = new float[3];
		float theta, phi, xy, nxy, u, v, sdif, cosb;

		float teXianglian1[2], teXianglian2[2];
		float vectorNorm, lamada1, lamada2;
		Point_t p, qNrm;

		ANN::ANNpoint      queryPt;
		ANN::ANNidxArray   nnIdx;
		ANN::ANNdistArray  dists;

		double epsKdtree = 0;
		queryPt = ANN::annAllocPt(3);

		nnIdx = new ANN::ANNidx[numNeighbor + 1];
		dists = new ANN::ANNdist[numNeighbor + 1];

		for (i = 0; i < numberPts; i++) {
			p = Pts[i]; qNrm = nrmPts[i];
			queryPt[0] = Pts[i].x;
			queryPt[1] = Pts[i].y;
			queryPt[2] = Pts[i].z;
			GkdTree->annkSearch(queryPt, numNeighbor + 1, nnIdx, dists, epsKdtree);
			for (j = 0; j < numNeighbor; j++)
				qArraySN[j] = nnIdx[j + 1];

			for (j = 0; j < numNeighbor; j++) {
				PtNeighbor[j].x = Pts[qArraySN[j]].x;
				PtNeighbor[j].y = Pts[qArraySN[j]].y;
				PtNeighbor[j].z = Pts[qArraySN[j]].z;
				//cout<<qArray[i]<<endl;
				nrmNeighbor[j].x = nrmPts[qArraySN[j]].x;
				nrmNeighbor[j].y = nrmPts[qArraySN[j]].y;
				nrmNeighbor[j].z = nrmPts[qArraySN[j]].z;
			}

			theta = acos(nrmPts[i].z);
			phi = atan2(nrmPts[i].y, nrmPts[i].x);
			ox[0] = -sin(phi);  ox[1] = cos(phi); ox[2] = 0;
			oy[0] = cos(phi)*cos(theta);
			oy[1] = sin(phi)*cos(theta);
			oy[2] = -sin(theta);
			for (j = 0; j < numNeighbor; j++) {
				jubuhua[0] = PtNeighbor[j].x - p.x;
				jubuhua[1] = PtNeighbor[j].y - p.y;
				jubuhua[2] = PtNeighbor[j].z - p.z;
				PtNeighbor[j].x = jubuhua[0] * ox[0] + jubuhua[1] * ox[1] + jubuhua[2] * ox[2];
				PtNeighbor[j].y = jubuhua[0] * oy[0] + jubuhua[1] * oy[1] + jubuhua[2] * oy[2];
				PtNeighbor[j].z = jubuhua[0] * nrmPts[i].x + jubuhua[1] * nrmPts[i].y + jubuhua[2] * nrmPts[i].z;

				jubuhua[0] = nrmNeighbor[j].x;
				jubuhua[1] = nrmNeighbor[j].y;
				jubuhua[2] = nrmNeighbor[j].z;
				nrmNeighbor[j].x = jubuhua[0] * ox[0] + jubuhua[1] * ox[1] + jubuhua[2] * ox[2];
				nrmNeighbor[j].y = jubuhua[0] * oy[0] + jubuhua[1] * oy[1] + jubuhua[2] * oy[2];
				nrmNeighbor[j].z = jubuhua[0] * nrmPts[i].x + jubuhua[1] * nrmPts[i].y + jubuhua[2] * nrmPts[i].z;
				vectorNorm = sqrtf(nrmNeighbor[j].x*nrmNeighbor[j].x + nrmNeighbor[j].y*nrmNeighbor[j].y +
					nrmNeighbor[j].z*nrmNeighbor[j].z);
				nrmNeighbor[j].x = nrmNeighbor[j].x / vectorNorm;
				nrmNeighbor[j].y = nrmNeighbor[j].y / vectorNorm;
				nrmNeighbor[j].z = nrmNeighbor[j].z / vectorNorm;
			}
			for (j = 0; j < 3; j++) {
				M[j][0] = 0; M[j][1] = 0; M[j][2] = 0; b[j] = 0;
			}
			for (j = 0; j < numNeighbor; j++) {
				xy = sqrt(PtNeighbor[j].x*PtNeighbor[j].x + PtNeighbor[j].y*PtNeighbor[j].y);
				if (fabs(xy) < 1.0e-21)
					xy = 1.e-21;
				u = PtNeighbor[j].x / xy;
				v = PtNeighbor[j].y / xy;
				M[0][0] = M[0][0] + u * u*u*u;
				M[0][1] = M[0][1] + 2 * u*u*u*v;
				M[0][2] = M[0][2] + u * u*v*v;
				M[1][1] = M[1][1] + 4 * u*u*v*v;
				M[1][2] = M[1][2] + 2 * u*v*v*v;
				M[2][2] = M[2][2] + v * v*v*v;

				nxy = nrmNeighbor[j].x*u + nrmNeighbor[j].y*v;
				if (fabs(nxy) < 1.0e-21)
					nxy = 1.0e-21;
				if ((PtNeighbor[j].z / xy - nrmNeighbor[j].z / nxy) < -(1.0e-21))
					sdif = -1;
				else if ((PtNeighbor[j].z / xy - nrmNeighbor[j].z / nxy) > (1.0e-21))
					sdif = 1;
				else
					sdif = 0;
				cosb = -sdif * nxy / fabs(nxy)*nrmNeighbor[j].z / sqrt(nrmNeighbor[j].z*nrmNeighbor[j].z + nxy * nxy);
				b[0] = b[0] + u * u*sdif*sqrt(1 - cosb * cosb) / xy;
				b[1] = b[1] + 2 * u*v*sdif*sqrt(1 - cosb * cosb) / xy;
				b[2] = b[2] + v * v*sdif*sqrt(1 - cosb * cosb) / xy;
			}
			M[1][0] = M[0][1]; M[2][0] = M[0][2]; M[2][1] = M[1][2];

			detM = M[0][0] * M[1][1] * M[2][2] + M[1][0] * M[2][1] * M[0][2] + M[2][0] * M[0][1] * M[1][2]
				- M[2][0] * M[1][1] * M[0][2] - M[1][0] * M[0][1] * M[2][2] - M[0][0] * M[2][1] * M[1][2];
			invM[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / detM;
			invM[0][1] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) / detM;
			invM[0][2] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / detM;
			invM[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / detM;
			invM[1][2] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) / detM;
			invM[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / detM;
			invM[1][0] = invM[0][1]; invM[2][0] = invM[0][2]; invM[2][1] = invM[1][2];

			A = invM[0][0] * b[0] + invM[0][1] * b[1] + invM[0][2] * b[2];
			B = invM[1][0] * b[0] + invM[1][1] * b[1] + invM[1][2] * b[2];
			C = invM[2][0] * b[0] + invM[2][1] * b[1] + invM[2][2] * b[2];

			lamada1 = (A + C + sqrtf((A - C)*(A - C) + 4 * B*B)) / 2;
			lamada2 = A + C - lamada1;
			if (lamada1 >= lamada2) {
				K1[i] = lamada1;
				K2[i] = lamada2;
			}
			else {
				K1[i] = lamada2;
				K2[i] = lamada1;
			}
			if (fabs(K1[i] - A) < 1.0e-16 && fabs(B) < 1.0e-16) {
				vectorNorm = sqrtf((K1[i] - C)*(K1[i] - C) + B * B);
				if (vectorNorm < 1.0e-16) {
					teXianglian1[0] = 1;
					teXianglian1[1] = 0;
				}
				else {
					teXianglian1[0] = (K1[i] - C) / vectorNorm;
					teXianglian1[1] = B / vectorNorm;
				}
			}
			else {
				vectorNorm = sqrtf((K1[i] - A)*(K1[i] - A) + B * B);
				if (vectorNorm < 1.0e-16) {
					teXianglian1[0] = 1;
					teXianglian1[1] = 0;
				}
				else {
					teXianglian1[0] = B / vectorNorm;
					teXianglian1[1] = (K1[i] - A) / vectorNorm;
				}
			}
			if (fabs(K2[i] - A) < 1.0e-16 && fabs(B) < 1.0e-16) {
				vectorNorm = sqrtf((K2[i] - C)*(K2[i] - C) + B * B);
				if (vectorNorm < 1.0e-16) {
					teXianglian2[0] = 0;
					teXianglian2[1] = 1;
				}
				else {
					teXianglian2[0] = (K2[i] - C) / vectorNorm;
					teXianglian2[1] = B / vectorNorm;
				}
			}
			else {
				vectorNorm = sqrtf((K2[i] - A)*(K2[i] - A) + B * B);
				if (vectorNorm < 1.0e-16) {
					teXianglian2[0] = 0;
					teXianglian2[1] = 1;
				}
				else {
					teXianglian2[0] = B / vectorNorm;
					teXianglian2[1] = (K2[i] - A) / vectorNorm;
				}
			}

			PriDirection1[i].x = teXianglian1[0] * ox[0] + teXianglian1[1] * oy[0];
			PriDirection1[i].y = teXianglian1[0] * ox[1] + teXianglian1[1] * oy[1];
			PriDirection1[i].z = teXianglian1[0] * ox[2] + teXianglian1[1] * oy[2];

			PriDirection2[i].x = teXianglian2[0] * ox[0] + teXianglian2[1] * oy[0];
			PriDirection2[i].y = teXianglian2[0] * ox[1] + teXianglian2[1] * oy[1];
			PriDirection2[i].z = teXianglian2[0] * ox[2] + teXianglian2[1] * oy[2];

		}
		for (i = 0; i < 3; i++)
			delete M[i];
		delete[]M;
		for (i = 0; i < 3; i++)
			delete invM[i];
		delete[]invM;
		delete[]b;
		delete[]ox;
		delete[]oy;
		delete[]jubuhua;
		delete[]qArraySN;
		delete[]nnIdx;
		delete[] dists;
		return 1;
	}

}