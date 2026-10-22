#include "dialogs/dialog_point_cloud_delaunay_reconstruction.h"

#include <module/core/point_cloud.h>
#include <module/core/surface_mesh.h>
#include <module/algo/point_cloud_delaunay_reconstruction.h>
#include <module/renderer/renderer.h>
#include <module/util/file_system.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

/*
* 设置操作界面中的按键
*/
DialogDelaunayReconstruction::DialogDelaunayReconstruction(MainWindow *window)
	: Dialog(window) {
	setupUi(this);
	//
	lineEdit_reconstruct_radius->setText("5.0");
	lineEdit_reconstruct_interations->setText("1");
	lineEdit_reconstruct_neighbors->setText("100");
	lineEdit_remesh_points->setText("30000");
	lineEdit_remesh_Lloyd_iter->setText("5");
	lineEdit_remesh_Newton_iter->setText("30");
	lineEdit_remesh_Newton_m->setText("7");

	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
	connect(checkBox_watertight_or_not, SIGNAL(clicked()), this, SLOT(checkRepairing()));
}

DialogDelaunayReconstruction::~DialogDelaunayReconstruction()
{
}

void DialogDelaunayReconstruction::checkRepairing()
{
	if (checkBox_watertight_or_not->isChecked())
	{
		lineEdit_reconstruct_radius->setText("5.0");
		lineEdit_reconstruct_neighbors->setText("100");
	}
	else
	{
		lineEdit_reconstruct_radius->setText("0.3");
		lineEdit_reconstruct_neighbors->setText("30");
	}
}
/*德洛内三角化*/
void DialogDelaunayReconstruction::apply()
{
	auto cloud = dynamic_cast<PointCloud*>(viewer_->currentModel());
	if ( !cloud )
	{
		LOG(ERROR) << "please input proper points to reconstruct mesh!";
		return;
	}
	const float radius = lineEdit_reconstruct_radius->text().toFloat();
	const int nb_iterations = lineEdit_reconstruct_interations->text().toInt();
	const int nb_neighbors = lineEdit_reconstruct_neighbors->text().toInt();
	const int nb_points = lineEdit_remesh_points->text().toInt();
	const int nb_Lloyd_iter = lineEdit_remesh_Lloyd_iter->text().toInt();
	const int nb_Newton_iter = lineEdit_remesh_Newton_iter->text().toInt();
	const int Newton_m = lineEdit_remesh_Newton_m->text().toInt();
	const bool bremeshing = checkBox_remesh_or_not->isChecked();
	const bool bwatertight = checkBox_watertight_or_not->isChecked();
	
	static DelnaunayReconstruction DR;
	SurfaceMesh * mesh = DR.reconstruct( cloud, radius, nb_iterations, nb_neighbors, nb_points, nb_Lloyd_iter, nb_Newton_iter, Newton_m, bremeshing, bwatertight );
	if (mesh) {
		const std::string &name = file_system::name_less_extension(cloud->name()) + "_delaunay_reconstruction";
		mesh->set_name(name);
		viewer_->addModel(mesh);
		window_->updateUi();
		viewer_->update();
	}
	this->close();
}