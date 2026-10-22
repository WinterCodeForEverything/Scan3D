#ifndef DIALOG_SURFACE_MESH_TEXTURING
#define DIALOG_SURFACE_MESH_TEXTURING

#include "dialog.h"
#include "ui_dialog_surface_mesh_texturing.h"


class DialogSurfaceMeshTexturing : public Dialog, public Ui::DialogSurfaceMeshTexturing {
	Q_OBJECT

public:
	DialogSurfaceMeshTexturing(MainWindow *window, QString & curDataDirectory );
	~DialogSurfaceMeshTexturing();

private Q_SLOTS:
	void openWorkingFolder();
	void openRelativeCameraPose();
	void saveTexturedFile();
	void chooseJPG();
	void choosePNG();
	void apply();

private:
	QString curDataDirectory_;
};

#endif  //DIALOG_SURFACE_MESH_TEXTURING