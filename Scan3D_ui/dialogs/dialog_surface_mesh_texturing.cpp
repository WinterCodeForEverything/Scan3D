#include "dialogs/dialog_surface_mesh_texturing.h"

#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_texturing.h>
#include <module/renderer/renderer.h>
#include <module/util/file_system.h>

#include <QDialogButtonBox>
#include <QFileDialog>
#include "paint_canvas.h"
#include "main_window.h"

using namespace scan3d;

DialogSurfaceMeshTexturing::DialogSurfaceMeshTexturing(MainWindow *window, QString &curDataDirectory )
	: Dialog(window), curDataDirectory_(curDataDirectory){
	setupUi(this);

	lineEdit_texturing_resolution_level->setText("0");
	lineEdit_texturing_min_resolution->setText("640");
	lineEdit_texturing_outlier_threshold->setText("0.06");
	lineEdit_texturing_ratio_data_smoothness->setText("0.1");
	lineEdit_texturing_size_mutiple->setText("0");
	lineEdit_texturing_rect_packing_heuristic->setText("3");
	lineEdit_texturing_empty_color->setText("0x00000000");
	lineEdit_texturing_max_threads->setText("4");

	connect(pushButton_open, SIGNAL(released()), this, SLOT(openWorkingFolder()));
	connect(pushButton_open_pose, SIGNAL(released()), this, SLOT(openRelativeCameraPose()));
	connect(pushButton_save, SIGNAL(released()), this, SLOT(saveTexturedFile()));
	connect(radioButton_jpg, SIGNAL(clicked()), this, SLOT(chooseJPG()));
	connect(radioButton_png, SIGNAL(clicked()), this, SLOT(choosePNG()));
	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()) );
}

DialogSurfaceMeshTexturing::~DialogSurfaceMeshTexturing()
{
	
}

/*打开工作路径，即含有图片的相机内参xml的文件夹*/
void DialogSurfaceMeshTexturing::openWorkingFolder()
{
	const QString& folderName = QFileDialog::getExistingDirectory(
		this,
		"Open images folder",
		curDataDirectory_
	);
	lineEdit_texturing_working_folder->setText(folderName);
	auto mesh = dynamic_cast<SurfaceMesh*>(viewer_->currentModel());
	if (!mesh || !file_system::is_file(mesh->name()))
		return;
	const std::string &saveName = file_system::name_less_extension(mesh->name()) + "_texture.obj";
	lineEdit_texturing_save_file->setText(QString(saveName.c_str()));
}

/*保存相机外参矩阵*/
void DialogSurfaceMeshTexturing::openRelativeCameraPose()
{
	const QString& openName = QFileDialog::getOpenFileName(
		this,
		"Open file",
		curDataDirectory_,
		"Supported formats (*.txt)\n"
		"All formats (*.*)"
	);
	lineEdit_texturing_relative_pose->setText(openName);
}

/*保存贴图后的模型及纹理图片*/
void DialogSurfaceMeshTexturing::saveTexturedFile()
{
	const QString& saveName = QFileDialog::getSaveFileName(
		this,
		"Save file",
		curDataDirectory_,
		"Supported formats (*.ply *.obj)\n"
		"All formats (*.*)"
	);
	lineEdit_texturing_save_file->setText(saveName);
}

/*选择用于贴图的照片类型为jpg*/
void DialogSurfaceMeshTexturing::chooseJPG()
{
	if (radioButton_jpg->isChecked())
		radioButton_png->setChecked(false);
	else
		radioButton_png->setChecked(true);
}

/*选择用于贴图的照片类型为png*/
void DialogSurfaceMeshTexturing::choosePNG()
{
	if (radioButton_png->isChecked())
		radioButton_jpg->setChecked(false);
	else
		radioButton_jpg->setChecked(true);
}

/*完成纹理贴图*/
void DialogSurfaceMeshTexturing::apply()
{
	auto mesh = dynamic_cast<SurfaceMesh*>(viewer_->currentModel());
	if (!mesh)
	{
		LOG(ERROR) << "please input proper mesh to texture!";
		return;
	}
	const std::string strWorkingFolder = lineEdit_texturing_working_folder->text().toUtf8().constData();
	const std::string strRelativePose = lineEdit_texturing_relative_pose->text().toUtf8().constData();
	const std::string strSaveFile = lineEdit_texturing_save_file->text().toUtf8().constData();
	const int nResolutionLevel = lineEdit_texturing_resolution_level->text().toInt();
	const int nMinResolution = lineEdit_texturing_min_resolution->text().toInt();
	const float fOutlierThreshold = lineEdit_texturing_outlier_threshold->text().toFloat();
	const float fRatioDataSmoothness = lineEdit_texturing_ratio_data_smoothness->text().toFloat();
	const bool bGlobalSeamLeveling = checkBox_texturing_global_seam_leveling->isChecked();
	const bool bLocalSeamLeveling = checkBox_texturing_local_seam_leveling->isChecked();
	const int nTextureSizeMultiple = lineEdit_texturing_size_mutiple->text().toInt();
	const int nRectPackingHeuristic = lineEdit_texturing_rect_packing_heuristic->text().toInt();
	const int nColEmpty = lineEdit_texturing_empty_color->text().toInt( bool(), 16 );
	const int nMaxThreads = lineEdit_texturing_max_threads->text().toInt();
	const bool bImageType = radioButton_jpg->isChecked();

	static SurfaceMeshTexturing texture;
	if (texture.texturing(mesh, strWorkingFolder, strRelativePose, strSaveFile, nResolutionLevel, nMinResolution, fOutlierThreshold, fRatioDataSmoothness,
			bGlobalSeamLeveling, bLocalSeamLeveling, nTextureSizeMultiple, nRectPackingHeuristic, nColEmpty, nMaxThreads, bImageType) )
	{
		const std::string &name = file_system::name_less_extension(mesh->name()) + "_texture";
		mesh->set_name(name);
		//viewer_->addModel(mesh);
		window_->updateUi();
		viewer_->update();
		this->close();
	}
}