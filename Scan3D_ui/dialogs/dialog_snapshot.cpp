#include "dialog_snapshot.h"

#include "main_window.h"
#include "paint_canvas.h"


using namespace scan3d;

DialogSnapshot::DialogSnapshot(MainWindow *window)
        : Dialog(window) {
    setupUi(this);
    layout()->setSizeConstraint(QLayout::SetFixedSize);

    const double scale = 1.0;
    doubleSpinBoxImageScale->setValue(scale);

    computeImageSize();;

    connect(viewer_, SIGNAL(resized()), this, SLOT(computeImageSize()));

    connect(doubleSpinBoxImageScale, SIGNAL(valueChanged(double)), this, SLOT(computeImageSize()));
    connect(pushButtonOK, SIGNAL(clicked()), this, SLOT(accept()));
    connect(pushButtonCancel, SIGNAL(clicked()), this, SLOT(reject()));
}


DialogSnapshot::~DialogSnapshot() {
}

/*计算图像大小*/
void DialogSnapshot::computeImageSize() {
    double scale = doubleSpinBoxImageScale->value();
    int w = static_cast<int>(viewer_->width() * viewer_->dpi_scaling() * scale);
    int h = static_cast<int>(viewer_->height() * viewer_->dpi_scaling() * scale);
    spinBoxImageWidth->setValue(w);
    spinBoxImageHeight->setValue(h);
}

/*保存截屏*/
void DialogSnapshot::saveSnapshot(const QString& fileName) {
    // disable ui to prevent the rendering from being modified.
    window_->setEnabled(false);

    // Hide closed dialog
    QApplication::processEvents();

    int w = spinBoxImageWidth->value();
    int h = spinBoxImageHeight->value();
    int samples = spinBoxSamples->value();
    viewer_->saveSnapshot(w, h, samples, fileName,
                          checkBoxUseWhiteBackground->isChecked(),
                          checkBoxExpandFrustum->isChecked()
    );

    // restore the ui
    window_->setEnabled(true);
}
