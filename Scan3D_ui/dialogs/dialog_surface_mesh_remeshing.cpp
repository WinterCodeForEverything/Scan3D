#include "dialog_surface_mesh_remeshing.h"

#include <module/core/surface_mesh.h>
#include <module/algo/surface_mesh_features.h>
#include <module/algo/surface_mesh_remeshing.h>
#include <module/renderer/renderer.h>

#include "main_window.h"
#include "paint_canvas.h"


using namespace scan3d;

DialogSurfaceMeshRemeshing::DialogSurfaceMeshRemeshing(MainWindow *window)
        : Dialog(window) {
    setupUi(this);

    comboBoxScheme->addItem("Uniform Remeshing");
    //comboBoxScheme->addItem("Adaptive Remeshing");
    comboBoxScheme->setCurrentIndex(0);

    connect(comboBoxScheme, SIGNAL(currentIndexChanged(const QString &)),this, SLOT(setRemeshingScheme(const QString &)));

    layout()->setSizeConstraint(QLayout::SetFixedSize);
    connect(okButton, SIGNAL(clicked()), this, SLOT(apply()));
}


DialogSurfaceMeshRemeshing::~DialogSurfaceMeshRemeshing() {
}


void DialogSurfaceMeshRemeshing::setRemeshingScheme(const QString &s) {
    if (s == "Uniform Remeshing") {
        labelEdgeLength->setEnabled(true);
        doubleSpinBoxEdgeLength->setEnabled(true);
    } else {
        labelEdgeLength->setEnabled(false);
        doubleSpinBoxEdgeLength->setEnabled(false);
    }
}


void DialogSurfaceMeshRemeshing::apply() {
    SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
    if (!mesh)
        return;

    if (checkBoxUseFeatures->isChecked()) {
        const int feature_angle = spinBoxDihedralAngle->value();
        SurfaceMeshFeatures sf(mesh);
        sf.clear();
        sf.detect_angle(feature_angle);
        sf.detect_boundary();
    }

    if (comboBoxScheme->currentText() == "Uniform Remeshing") {
        float len(0.0f);
        for (auto eit : mesh->edges())
            len += distance(mesh->position(mesh->vertex(eit, 0)),
                            mesh->position(mesh->vertex(eit, 1)));
        len /= (float) mesh->n_edges();
        SurfaceMeshRemeshing(mesh).uniform_remeshing(len * doubleSpinBoxEdgeLength->value());
    } else { // Adaptive remeshing
        auto bb = mesh->bounding_box().diagonal();
        SurfaceMeshRemeshing(mesh).adaptive_remeshing(
                0.001 * bb,  // min length
                0.100 * bb,  // max length
                0.001 * bb); // approx. error
    }

    mesh->renderer()->update();
    viewer_->update();
    window_->updateUi();
}

