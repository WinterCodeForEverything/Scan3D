#include "dialogs/dialog_point_cloud_feature_FPFH.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_feature_FPFH.h>
#include <module/renderer/renderer.h>

#include <QFileDialog>
#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogPointCloudComputeFeatureFPFH::DialogPointCloudComputeFeatureFPFH(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_featureFPFH_pointnumber->setText("10");
	lineEdit_featureFPFH_radius->setText("0.1");
	lineEdit_featureFPFH_savenumber->setText("50");

	connect(saveButton, SIGNAL(clicked()), this, SLOT(save()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogPointCloudComputeFeatureFPFH::~DialogPointCloudComputeFeatureFPFH()
{
}

/*
	计算并保存点云的FPFH特征
*/

void DialogPointCloudComputeFeatureFPFH::save()
{
	auto cloud = dynamic_cast<PointCloud*>(viewer_->currentModel());
	if ( !cloud )
	{
		LOG(ERROR) << "please input proper points to statistic filter";
		return;
	}
	const int pointsNum = lineEdit_featureFPFH_pointnumber->text().toInt();
	const float radius = lineEdit_featureFPFH_radius->text().toFloat();
	const int saveNum = lineEdit_featureFPFH_savenumber->text().toInt();

	const QString& fileName = QFileDialog::getSaveFileName(
		this,
		"Save file(s)",
		QString::fromStdString(""),
		"Supported formats (*.txt)"
	);

	PointCloudFeatureFPFH pointcloudfeatureFPFH;
	pointcloudfeatureFPFH.SaveFPFHFeature( cloud, pointsNum, (double)radius, fileName.toStdString(), saveNum );
	cloud->renderer()->update();
	viewer_->update();
	window_->updateStatusBar();
	this->close();
}