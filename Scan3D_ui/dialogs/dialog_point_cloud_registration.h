#ifndef DIALOG_POINT_CLOUD_REGISTRATION_H
#define DIALOG_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include "dialog.h"
#include "ui_dialog_point_cloud_registration.h"

class DialogPointCloudRegistration : public Dialog, public Ui::DialogPointCloudRegistration {
	Q_OBJECT

public:
	DialogPointCloudRegistration(MainWindow *window, QString & curDataDirectory );
	~DialogPointCloudRegistration();

	void updatePointCloudList();

private Q_SLOTS:
	void apply();
	void addPointCloud();
	void deletePointCloud();

	void modleregistration();
	void ICP_Princeton();

	void savePoses();
	void checkSaveButton();

private:
	std::vector<int> m_selectedPointCloudId;
	QString curDataDirectory_;

};

#endif  //DIALOG_POINT_CLOUD_REGISTRATION_H