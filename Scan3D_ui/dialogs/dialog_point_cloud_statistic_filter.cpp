#include "dialogs/dialog_point_cloud_statistic_filter.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_filter_utils.h>
#include <module/renderer/renderer.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogPointCloudStatisticFilter::DialogPointCloudStatisticFilter(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_statisticFilter_nSigma->setText("1.0");
	lineEdit_statisticFilter_numberpoint->setText("6");

	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogPointCloudStatisticFilter::~DialogPointCloudStatisticFilter()
{
}

/*统计滤波*/
void DialogPointCloudStatisticFilter::apply()
{
	auto model = viewer_->currentModel();
	if (!model || !dynamic_cast<PointCloud*>(model))
	{
		LOG(ERROR) << "please input proper points to statistic filter";
		return;
	}
		
	const float sigma = lineEdit_statisticFilter_nSigma->text().toFloat();
	const int pointsNum = lineEdit_statisticFilter_numberpoint->text().toInt();
	FilterUtils::RemoveStatisticalOutliers(dynamic_cast<PointCloud *>(model), pointsNum, sigma );
	model->renderer()->update();
	viewer_->update();
	window_->updateStatusBar();
	this->close();
}