#ifndef DIALOG_POISSON_RECONSTRUCTION_H
#define DIALOG_POISSON_RECONSTRUCTION_H

#include "dialog.h"
#include "ui_dialog_poisson_reconstruction.h"


class PaintCanvas;
class MainWindow;

class DialogPoissonReconstruction : public Dialog, public Ui::DialogPoissonReconstruction
{
    Q_OBJECT

public:
    explicit DialogPoissonReconstruction(MainWindow *window);
    ~DialogPoissonReconstruction();

private Q_SLOTS:
    void showHint(bool);
    void resetParameters();
    void reconstruct();
    void trim();

private:
    int			default_octree_depth_;
    int         default_samples_per_node_;
    double		default_trim_value_;
    double		default_area_ratio_;

    std::string density_attr_name_;
};

#endif // DIALOG_POISSON_RECONSTRUCTION_H
