#ifndef DIALOG_DELAUNAY_RECONSTRUCTION_H
#define DIALOG_DELAUNAY_RECONSTRUCTION_H

#include "dialog.h"
#include "ui_dialog_point_cloud_delaunay_reconstruction.h"


class DialogDelaunayReconstruction : public Dialog, public Ui::DialogDelaunayReconstruction {
	Q_OBJECT

public:
	DialogDelaunayReconstruction(MainWindow *window);
	~DialogDelaunayReconstruction();

private Q_SLOTS:
	void apply();
	void checkRepairing();
};

#endif  //DIALOG_DELAUNAY_RECONSTRUCTION_H