#ifndef DIALOG_SURFACE_REMESHING_H
#define DIALOG_SURFACE_REMESHING_H

#include "dialog.h"
#include "ui_dialog_surface_mesh_remeshing.h"


class DialogSurfaceMeshRemeshing : public Dialog, public Ui::DialogSurfaceMeshRemeshing {
Q_OBJECT

public:
    explicit DialogSurfaceMeshRemeshing(MainWindow *window);
    ~DialogSurfaceMeshRemeshing();

private Q_SLOTS:
    void apply();
    void setRemeshingScheme(const QString &);
};

#endif // DIALOG_SURFACE_REMESHING_H
