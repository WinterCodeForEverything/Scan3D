#ifndef DIALOG_POINT_CLOUD_BILATERAL_FILTER_H
#define DIALOG_POINT_CLOUD_BILATERAL_FILTER_H

#include "dialog.h"
#include "ui_dialog_point_cloud_bilateral_filter.h"

class DialogPointCloudBilateralFilter : public Dialog, public Ui::DialogPointCloudBilateralFilter {
	Q_OBJECT

public:
	DialogPointCloudBilateralFilter(MainWindow *window);
	~DialogPointCloudBilateralFilter();

private Q_SLOTS:
	void apply();
};

#endif  //DIALOG_POINT_CLOUD_BILATERAL_FILTER_H