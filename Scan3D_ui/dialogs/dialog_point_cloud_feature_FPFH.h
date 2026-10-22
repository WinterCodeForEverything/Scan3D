#ifndef DIALOG_POINT_CLOUD_FEATURE_FPFH_H
#define DIALOG_POINT_CLOUD_FEATURE_FPFH_H

#include "dialog.h"
#include "ui_dialog_point_cloud_feature_FPFH.h"

class DialogPointCloudComputeFeatureFPFH : public Dialog, public Ui::DialogPointCloudComputeFeatureFPFH {
	Q_OBJECT

public:
	DialogPointCloudComputeFeatureFPFH(MainWindow *window);
	~DialogPointCloudComputeFeatureFPFH();

private Q_SLOTS:
	void save();
};

#endif //DIALOG_POINT_CLOUD_FEATURE_FPFH_H