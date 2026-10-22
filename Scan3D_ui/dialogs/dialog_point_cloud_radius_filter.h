#ifndef DIALOG_POINT_CLOUD_RADIUS_FILTER_H
#define DIALOG_POINT_CLOUD_RADIUS_FILTER_H

#include "dialog.h"
#include "ui_dialog_point_cloud_radius_filter.h"

class DialogPointCloudRadiusFilter : public Dialog, public Ui::DialogPointCloudRadiusFilter {
	Q_OBJECT

public:
	DialogPointCloudRadiusFilter(MainWindow *window);
	~DialogPointCloudRadiusFilter();

private Q_SLOTS:
	void apply();

private:
	double default_radius_;
	int points_number_;
};

#endif  //DIALOG_POINT_CLOUD_RADIUS_FILTER_H