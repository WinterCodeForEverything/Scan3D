#include "dialogs/dialog_point_cloud_curvation_estimate.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_curvation_estimate.h>
#include <module/renderer/renderer.h>
#include <module/renderer/drawable_points.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogPointCloudCurvationEstimate::DialogPointCloudCurvationEstimate(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_curvationEstimate_numberpoint->setText("20");

	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogPointCloudCurvationEstimate::~DialogPointCloudCurvationEstimate()
{
}
/*曲率计算*/
void DialogPointCloudCurvationEstimate::apply()
{
	PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
	if ( !cloud )
	{
		LOG(ERROR) << "please input proper points to estimate curvature";
		return;
	}
	//std::vector<vec3>& pointCloudColor = cloud->vertex_property("v:color", vec3()).vector();
	cloud->renderer()->update();
	viewer_->update();
	const int pointsNum = lineEdit_curvationEstimate_numberpoint->text().toInt();
	PointCloudCurvationEstimate pointcloudcurvationestimate;
	auto vvCurval = pointcloudcurvationestimate.curvation_estimate( cloud, pointsNum );
	std::vector<float> vGaussianCur(vvCurval.size()), vk1Cur(vvCurval.size()), vk2Cur(vvCurval.size());
	std::vector<double> vpointCur;
	std::vector<vec3> tmpColor(vvCurval.size());
	double dmax = 0.0, tmpGaussianCur;
	for( int i = 0; i < vvCurval.size(); i++ )
	{ 
		vpointCur = vvCurval[i];
		tmpGaussianCur = abs( vpointCur[0] * vpointCur[1] );
		dmax = ( dmax > tmpGaussianCur ? dmax : tmpGaussianCur );
		vGaussianCur[i] = tmpGaussianCur;
		vk1Cur[i] = abs(vpointCur[0]);
		vk2Cur[i] = abs(vpointCur[1]);
		tmpColor[i] = vec3(1, 1, 1);
	}

	//for (int i = 0; i < pointCloudColor.size(); i++)
	//	pointCloudColor[i] = vec3(vGaussianCur[i], vGaussianCur[i], 0);
	cloud->vertex_property("v:curvature_gaussian", float()).vector() = vGaussianCur;
	cloud->vertex_property("v:curvature_k1", float()).vector() = vk1Cur;
	cloud->vertex_property("v:curvature_k2", float()).vector() = vk2Cur;


	cloud->renderer()->update();
	viewer_->update();
	window_->updateRenderingPanel();
	this->close();
}