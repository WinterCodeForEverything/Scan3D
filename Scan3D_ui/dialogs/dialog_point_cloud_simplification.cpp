#include "dialogs/dialog_point_cloud_simplification.h"

#include <module/kdtree/kdtree_search_eth.h>
#include <module/algo/point_cloud_simplification.h>
#include <module/renderer/renderer.h>
#include <module/util/logging.h>
#include <module/util/stop_watch.h>
#include <QButtonGroup>
#include <QIntValidator>

#include "paint_canvas.h"
#include "main_window.h"


using namespace scan3d;


/*
* 设置操作界面中的按键
*/
DialogPointCloudSimplification::DialogPointCloudSimplification(MainWindow *window)
        : Dialog(window), kdtree_(nullptr) {
    setupUi(this);
    layout()->setSizeConstraint(QLayout::SetFixedSize);

    // default value
    lineEditDistanceThreshold->setText("0.01");
    lineEditAverageSpacing->setText("unknown");
    lineEditExpectedPointNumber->setText("100000");
    lineEditExpectedPointNumber->setValidator(new QIntValidator(1, 1000000000, this));

    QButtonGroup *buttonGroup = new QButtonGroup(this);
    buttonGroup->addButton(radioButtonExpectedPointNumber, 0);
    buttonGroup->addButton(radioButtonDistanceThreshold, 1);
    connect(buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(strategyChanged(int)));
    strategyChanged(0);

    connect(buttonComputeAvgSpacing, SIGNAL(clicked()), this, SLOT(computeAvgSpacing()));
    connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
    connect(qureyButton, SIGNAL(clicked()), this, SLOT(query()));
}


DialogPointCloudSimplification::~DialogPointCloudSimplification() {
    if (kdtree_)
        delete kdtree_;
}


void DialogPointCloudSimplification::closeEvent(QCloseEvent *e) {
    points_to_remove_.clear();
    QDialog::closeEvent(e);
}


void DialogPointCloudSimplification::showEvent(QShowEvent *e) {
    PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
    if (cloud) {
        int num = cloud->n_vertices();
        lineEditExpectedPointNumber->setText(QString("%1").arg(num));
    }
    QDialog::showEvent(e);
}

/*选择不同采样策略，点数或者平均距离*/
void DialogPointCloudSimplification::strategyChanged(int id) {
    if (id == 0) {
        lineEditExpectedPointNumber->setDisabled(false);
        lineEditDistanceThreshold->setDisabled(true);
        checkBoxUniform->setDisabled(true);
        lineEditAverageSpacing->setDisabled(true);
        buttonComputeAvgSpacing->setDisabled(true);
    } else {
        lineEditExpectedPointNumber->setDisabled(true);
        lineEditDistanceThreshold->setDisabled(false);
        checkBoxUniform->setDisabled(false);
        lineEditAverageSpacing->setDisabled(false);
        buttonComputeAvgSpacing->setDisabled(false);
    }
}

/*建立kdtree*/
void DialogPointCloudSimplification::constructKdTree() {
    PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
    if (cloud) {
        if (kdtree_)
            delete kdtree_;
        kdtree_ = new KdTreeSearch_ETH;
        kdtree_->begin();
        kdtree_->add_point_cloud(cloud);
        kdtree_->end();
    }
}

/*计算点间平均距离*/
void DialogPointCloudSimplification::computeAvgSpacing() {
    PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
    if (cloud) {
        constructKdTree();

        if (kdtree_) {
            float as = PointCloudSimplification::average_spacing(cloud, kdtree_, 6);
            lineEditAverageSpacing->setText(QString("%1").arg(as));
            show();
        }
    }
}

/*查寻需要删除的点*/
void DialogPointCloudSimplification::query() {
    PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
    if (cloud) {
        if (radioButtonExpectedPointNumber->isChecked()) {
            unsigned int expected_number = lineEditExpectedPointNumber->text().toInt();
            points_to_remove_ = PointCloudSimplification::uniform_simplification(cloud, expected_number);
        } else {
            float threshold = lineEditDistanceThreshold->text().toFloat();
            if (checkBoxUniform->isChecked()) {
                if (!kdtree_)
                    constructKdTree();
                points_to_remove_ = PointCloudSimplification::uniform_simplification(cloud, threshold, kdtree_);
            } else
                points_to_remove_ = PointCloudSimplification::grid_simplification(cloud, threshold);
        }
        LOG(INFO) << cloud->n_vertices() - points_to_remove_.size() << " points will remain";
    }
}

/*完成下采样*/
void DialogPointCloudSimplification::apply() {
    PointCloud *cloud = dynamic_cast<PointCloud *>(viewer_->currentModel());
    if (cloud) {
        if (points_to_remove_.size() == 0) {
            LOG(INFO) << "please query points that can be removed";
            return;
        }

        int old_num = cloud->n_vertices();
        for (auto id : points_to_remove_)
            cloud->delete_vertex(PointCloud::Vertex(id));

        cloud->garbage_collection();

        if (!points_to_remove_.empty()) {
            points_to_remove_.clear();
            int new_num = cloud->n_vertices();
            LOG(INFO) << old_num - new_num << " points removed. " << new_num << " points remain";

            cloud->renderer()->update();
            viewer_->update();
        }

        delete kdtree_;
        kdtree_ = 0;
    }
}
