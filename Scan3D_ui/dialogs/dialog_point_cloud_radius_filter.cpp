#include "dialogs/dialog_point_cloud_radius_filter.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_filter_radius.h>
#include <module/renderer/renderer.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogPointCloudRadiusFilter::DialogPointCloudRadiusFilter(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_radiusFilter_radius->setText("0.001");
	lineEdit_radiusFilter_numberpoint->setText("20");

	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogPointCloudRadiusFilter::~DialogPointCloudRadiusFilter()
{
	
}
/*半径滤波*/
void DialogPointCloudRadiusFilter::apply()
{
	auto model = viewer_->currentModel();
	if (!model || !dynamic_cast<PointCloud*>(model))
	{
		LOG(ERROR) << "please input proper points to radius filter";
		return;
	}
	const float radius = lineEdit_radiusFilter_radius->text().toFloat();
	const int pointsNum = lineEdit_radiusFilter_numberpoint->text().toInt();
	FilterRadius::RemoveRadiusOutliers(dynamic_cast<PointCloud *>(model), pointsNum, radius );
	model->renderer()->update();
	viewer_->update();
	window_->updateStatusBar();
	this->close();
}