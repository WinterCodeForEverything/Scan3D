#ifndef DIALOG_POINT_CLOUD_STATISTIC_FILTER_H
#define DIALOG_POINT_CLOUD_STATISTIC_FILTER_H

#include "dialog.h"
#include "ui_dialog_point_cloud_statistic_filter.h"

class DialogPointCloudStatisticFilter : public Dialog, public Ui::DialogPointCloudStatisticFilter {
	Q_OBJECT

public:
	DialogPointCloudStatisticFilter(MainWindow *window);
	~DialogPointCloudStatisticFilter();

private Q_SLOTS:
	void apply();

private:
	double default_sigma_;
	int points_number_;
};

#endif //DIALOG_POINT_CLOUD_STATISTIC_FILTER_H