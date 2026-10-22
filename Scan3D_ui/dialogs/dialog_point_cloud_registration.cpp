#include <algorithm>
#include "dialogs/dialog_point_cloud_registration.h"

#include <module/core/point_cloud.h>
#include <module/algo/point_cloud_registration.h>
#include <module/renderer/renderer.h>
#include <module/util/file_system.h>

#include <QDialogButtonBox>
#include "paint_canvas.h"
#include "main_window.h"
#include <QFileDialog>
#include <iostream>
#include<fstream>
#include<QDebug>
using namespace scan3d;

DialogPointCloudRegistration::DialogPointCloudRegistration(MainWindow *window, QString & curDataDirectory )
	: Dialog(window), curDataDirectory_(curDataDirectory) {
	setupUi(this);

	connect(pushButton_registration_add, SIGNAL(clicked()), this, SLOT(addPointCloud()));
	connect(pushButton_registration_delete, SIGNAL(clicked()), this, SLOT(deletePointCloud()));
	connect(pushButton_registration_apply, SIGNAL(clicked()), this, SLOT(apply()));
	connect(pushButton_registration_cancel, SIGNAL(clicked()), this, SLOT(close()));
	//connect(pushButton_Error, SIGNAL(clicked()), this, SLOT(error()));
	//connect(pushButton_transform, SIGNAL(clicked()), this, SLOT(transform()));
	connect(pushButton_modleregistration, SIGNAL(clicked()), this, SLOT(modleregistration()));
	//connect(pushButton_anasys, SIGNAL(clicked()), this, SLOT(anasys()));
	connect(pushButton_ICPregistration, SIGNAL(clicked()), this, SLOT(ICP_Princeton()));
	connect(pushButton_saveposes, SIGNAL(clicked()), this, SLOT(savePoses()));
	connect(checkBox_saveposes, SIGNAL(clicked()), this, SLOT(checkSaveButton()));
	if (checkBox_saveposes->isChecked())
		pushButton_saveposes->setEnabled(true);
	else
		pushButton_saveposes->setEnabled(false);
}

DialogPointCloudRegistration::~DialogPointCloudRegistration()
{

	
}

void DialogPointCloudRegistration::updatePointCloudList()
{
	listWidget_registration_itemList->clear();
	listWidget_registration_selectedList->clear();
	m_selectedPointCloudId.clear();
	auto models = viewer_->models();
	for (auto model : models)
	{
		auto cloud = dynamic_cast<PointCloud*>(model);
		if (!cloud) continue;

		listWidget_registration_itemList->addItem(QString::fromStdString(file_system::base_name(model->name())));
	}
}



void DialogPointCloudRegistration::addPointCloud()
{
	QList<QListWidgetItem *> selectedItems = listWidget_registration_itemList->selectedItems();
	for (auto item : selectedItems)
	{
		int row = listWidget_registration_itemList->row(item);
		if (std::find(m_selectedPointCloudId.begin(), m_selectedPointCloudId.end(), row ) == m_selectedPointCloudId.end() )
		{
			listWidget_registration_selectedList->addItem(item->text());
			m_selectedPointCloudId.push_back(row);
		}
	}
}
void DialogPointCloudRegistration::deletePointCloud()
{
	QList<QListWidgetItem *> deleteItems = listWidget_registration_selectedList->selectedItems();
	for (auto item : deleteItems)
	{
		int row = listWidget_registration_selectedList->row(item);
		m_selectedPointCloudId.erase(m_selectedPointCloudId.begin() + row);
		listWidget_registration_selectedList->removeItemWidget(item);
		delete item;
	}

}
void DialogPointCloudRegistration::ICP_Princeton()
{
	registration_parameter para;
	para.noise_bin_rate = lineEdit_noise_bin_rate->text().toDouble();
	para.noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_bin_rate = lineEdit_normal_bin_rate->text().toDouble();
	para.normal_noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_angle = lineEdit_normal_angle->text().toDouble();
	para.normal_direcation = comboBox_normal_direcation->currentIndex();
	para.g_optimization_bin_rate = lineEdit_opt_bin_rate->text().toDouble();
	para.g_optimization_times = lineEdit_opt_times->text().toInt();

	para.turntable_voxel_size = 0.01; //lineEdit_turntable_voxel_size->text().toDouble();//下采样体素步长：指点云体素网格化过程中体素的边长大小，
	para.turntable_effective_area = 0.1;// lineEdit_turntable_effective_area->text().toDouble();//用于ICP重叠区域的收索
	para.turntable_rotation_angle = lineEdit_turntable_rotation_angle->text().toDouble();//有效旋转角度，判断变换矩阵的正确性
	para.turntable_translation_distance = lineEdit_turntable_translation_distance->text().toDouble();//有效平移距离，判断变换矩阵的正确性
	para.turntable_effective_area = para.turntable_translation_distance+0.1;
	if (para.turntable_effective_area > 0.3) para.turntable_effective_area = 0.3;
	QString str = "";
	QStringList strl = str.split(" ");

	for (int i = 0; i < strl.size(); i++)
	{
		QString str = strl[i].simplified();
		
	}
	//return;
	if (m_selectedPointCloudId.size() < 2)
	{
		LOG(ERROR) << "please select proper points to finish registration";
		return;
	}

	auto models = viewer_->models();
	std::vector<PointCloud*> vPointCloud;
	for (int id : m_selectedPointCloudId)
	{
		auto cloud = dynamic_cast<PointCloud*>(models[id]);
		vPointCloud.push_back(cloud);
	}
	std::vector<PointCloud*> newPointCloud;
	std::vector<Matrix4d> pose;
	PointCloudRegistration::Registration_ICP_Princeton(vPointCloud, newPointCloud, para, pose);

	
	for (int i = 0; i < newPointCloud.size(); i++)
	{
		viewer_->addModel(newPointCloud[i]);
		viewer_->fitScreen(newPointCloud[i]);
	}
	window_->updateUi();
	viewer_->update();

	this->close();
}
void DialogPointCloudRegistration::modleregistration()
{
	registration_parameter para;
	para.down_sampling_rate = lineEdit_downsamplerate->text().toDouble();
	//para.binning_size = lineEdit_votingbinningsize->text().toDouble();
	para.PPF_distance = lineEdit_PPF_distance->text().toDouble();
	para.PPF_angle = lineEdit_PPF_angle->text().toDouble();
	para.referencepoint_rate = lineEdit_reference_rate->text().toDouble();

	

	para.noise_bin_rate = lineEdit_noise_bin_rate->text().toDouble();
	para.noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_bin_rate = lineEdit_normal_bin_rate->text().toDouble();
	para.normal_noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_angle = lineEdit_normal_angle->text().toDouble();
	para.normal_direcation = comboBox_normal_direcation->currentIndex();
	para.g_optimization_bin_rate = lineEdit_opt_bin_rate->text().toDouble();
	para.g_optimization_times = lineEdit_opt_times->text().toInt();


	if (m_selectedPointCloudId.size() < 2)
	{
		LOG(ERROR) << "please select proper points to finish model registration for saving pose matrix!";
		return;
	}

	if (!checkBox_saveposes->isChecked())
	{
		LOG(ERROR) << "please check the saving bottun and select proper file path to save pose matrix!";
		return;
	}

	auto models = viewer_->models();
	std::vector<PointCloud*> vPointCloud;
	for (int id : m_selectedPointCloudId)
	{
		auto cloud = dynamic_cast<PointCloud*>(models[id]);
		vPointCloud.push_back(cloud);
	}
	std::vector<PointCloud*> newPointCloud;
	std::vector<Matrix4d> pose;
	PointCloudRegistration::Registration_fine(vPointCloud, newPointCloud, para, pose);

	//pose.insert(pose.begin(), 1, Matrix4d::Identity());
	std::vector<std::string> vPoseName;
	for (auto p : vPointCloud)
		vPoseName.push_back(file_system::base_name(p->name()));
	vPoseName.erase(vPoseName.begin() );

	std::string strFilePath = lineEdit_savefilepath->text().toUtf8().constData();
	if (!PointCloudRegistration::savePoses(vPoseName, pose, strFilePath))
	{
		LOG(ERROR) << "Can't open the saved file, please check the file path!";
		return;
	}
	this->close();
}

void DialogPointCloudRegistration::checkSaveButton()
{
	if (checkBox_saveposes->isChecked())
		pushButton_saveposes->setEnabled(true);
	else
		pushButton_saveposes->setEnabled(false);
}

void DialogPointCloudRegistration::savePoses()
{
	const QString& saveName = QFileDialog::getSaveFileName(
		this,
		"Save file(s)",
		curDataDirectory_,
		"Supported formats (*.txt)\n"
		"All formats (*.*)"
	);
	lineEdit_savefilepath->setText(saveName);
}


void DialogPointCloudRegistration::apply()
{
	registration_parameter para;
	para.down_sampling_rate = lineEdit_downsamplerate->text().toDouble();
	//para.binning_size = lineEdit_votingbinningsize->text().toDouble();
	para.PPF_distance = lineEdit_PPF_distance->text().toDouble();
	para.PPF_angle = lineEdit_PPF_angle->text().toDouble();
	para.referencepoint_rate = lineEdit_reference_rate->text().toDouble();


	para.noise_bin_rate = lineEdit_noise_bin_rate->text().toDouble();
	para.noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_bin_rate=lineEdit_normal_bin_rate->text().toDouble();
	para.normal_noise_max_rate = lineEdit_normal_noise_max_rate->text().toDouble();
	para.normal_angle = lineEdit_normal_angle->text().toDouble();
	para.normal_direcation = comboBox_normal_direcation->currentIndex();
	para.g_optimization_bin_rate= lineEdit_opt_bin_rate->text().toDouble();
	para.g_optimization_times= lineEdit_opt_times->text().toInt();


	

//return;
	if (m_selectedPointCloudId.size() < 2)
	{
		LOG(ERROR) << "please select proper points to finish registration";
		return;
	}
	
	auto models = viewer_->models();
	std::vector<PointCloud*> vPointCloud;
	for (int id : m_selectedPointCloudId)
	{
		auto cloud = dynamic_cast<PointCloud*>(models[id]);
		vPointCloud.push_back(cloud);
	}
	std::vector<PointCloud*> newPointCloud;
	std::vector<Matrix4d> pose;
	PointCloudRegistration::Registration(vPointCloud, newPointCloud, para, pose);

	for (int i=0;i< newPointCloud.size();i++)
	{
		viewer_->addModel(newPointCloud[i]);
		viewer_->fitScreen(newPointCloud[i]);
	}
	window_->updateUi();
	viewer_->update();

	//save poses
	if (checkBox_saveposes->isChecked())
	{
		pose.insert(pose.begin(), 1, Matrix4d::Identity());
		std::vector<std::string> vPoseName;
		for (auto p : vPointCloud)
			vPoseName.push_back(file_system::base_name(p->name()));

		std::string strFilePath = lineEdit_savefilepath->text().toUtf8().constData();
		if (!PointCloudRegistration::savePoses(vPoseName, pose, strFilePath))
		{
			LOG(ERROR) << "Can't open the saved file, please check the file path!";
			return;
		}
	}

	this->close();
}