#include "dialogs/dialog_point_cloud_bilateral_filter.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_bilateral_filter.h>
#include <module/renderer/renderer.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogPointCloudBilateralFilter::DialogPointCloudBilateralFilter(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_bilateralFilter_radius->setText("0.01");
	lineEdit_bilateralFilter_normalRatius->setText("0.8");
	lineEdit_bilateralFilter_interactionTimes->setText( "3" );

	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogPointCloudBilateralFilter::~DialogPointCloudBilateralFilter()
{
	
}
/*双边滤波*/
void DialogPointCloudBilateralFilter::apply()
{
	auto model = viewer_->currentModel();
	if (!model || !dynamic_cast<PointCloud*>(model))
	{
		LOG(ERROR) << "please input proper points to radius filter";
		return;
	}
	const float radius = lineEdit_bilateralFilter_radius->text().toFloat();
	const float normal_radius = lineEdit_bilateralFilter_normalRatius->text().toFloat();
	const int interation_times = lineEdit_bilateralFilter_interactionTimes->text().toInt();

	TPointCloudBilateralFilter::applyBilateralFilterIteration(dynamic_cast<PointCloud *>(model), radius, normal_radius, interation_times );
	model->renderer()->update();
	viewer_->update();
	window_->updateStatusBar();
	this->close();
}