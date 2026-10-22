#ifndef DIALOG_POINT_CLOUD_CURVATION_ESTIMATE_H
#define DIALOG_POINT_CLOUD_CURVATION_ESTIMATE_H

#include "dialog.h"
#include "ui_dialog_point_cloud_curvation_estimate.h"

class DialogPointCloudCurvationEstimate : public Dialog, public Ui::DialogPointCloudCurvationEstimate {
	Q_OBJECT

public:
	DialogPointCloudCurvationEstimate(MainWindow *window);
	~DialogPointCloudCurvationEstimate();

private Q_SLOTS:
	void apply();

private:
	int points_number_;
};

#endif //DIALOG_POINT_CLOUD_CURVATION_ESTIMATE_H