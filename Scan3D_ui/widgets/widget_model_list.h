#ifndef WIDGET_MODEL_LIST_H
#define WIDGET_MODEL_LIST_H

#include <QTreeWidget>

namespace scan3d {
    class Model;
}

class MainWindow;
class PaintCanvas;

class WidgetModelList : public QTreeWidget {
Q_OBJECT

public:
    WidgetModelList(QWidget *parent = 0);
    ~WidgetModelList();

    void init(MainWindow *w);

    // single model
    void addModel(scan3d::Model *obj, bool make_current);

    void deleteModel(scan3d::Model *obj, bool fit);

    void mergeModels(const std::vector<scan3d::Model *> &objs);

    void decomposeModel(scan3d::Model *obj);

    void updateModelList();

    void prepareContextMenu(QMenu *menu);

private Q_SLOTS :

    void setAutoFocus(bool b);
    void setSelectedOnly(bool b);

    void duplicateCurrent();
    void mergeSelected();
    void deleteSelected();
    void decomposeSelected();

    void invertSelection();
    void showSelected();
    void hideSelected();
    void invertShowHide();

    void showAllModels();

    void modelItemPressed(QTreeWidgetItem *current, int column);
    void currentModelItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
    void modelItemSelectionChanged();

    void showContextMenu(const QPoint &p);

protected:
    void mousePressEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);

private:
    void hideOtherModels(scan3d::Model *model);

    PaintCanvas *viewer();

private:
    MainWindow *mainWindow_;

    QMenu *popupMenu_;

    bool auto_focus_;
    bool selected_only_;
};

#endif