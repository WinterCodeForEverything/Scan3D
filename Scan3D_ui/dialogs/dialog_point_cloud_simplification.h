#ifndef DIALOG_SAMPLING_RESOLUTION_H
#define DIALOG_SAMPLING_RESOLUTION_H

#include <module/core/point_cloud.h>

#include "dialog.h"
#include "ui_dialog_point_cloud_simplification.h"


namespace scan3d {
    class KdTreeSearch;
}

class DialogPointCloudSimplification : public Dialog, public Ui::DialogPointCloudSimplification
{
	Q_OBJECT

public:
	DialogPointCloudSimplification(MainWindow *window);
	~DialogPointCloudSimplification();

private Q_SLOTS:
	void strategyChanged(int id);

	void computeAvgSpacing();
	void apply();
	void query();

protected:
	virtual void closeEvent(QCloseEvent* e);
	virtual void showEvent(QShowEvent* e);

	void constructKdTree();

private:
	scan3d::KdTreeSearch*    kdtree_;
	std::vector<scan3d::PointCloud::Vertex> points_to_remove_;
};

#endif // DLG_SAMPLING_RESOLUTION_H
