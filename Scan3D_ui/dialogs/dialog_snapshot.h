#ifndef DIALOG_SNAPSHOT_H
#define DIALOG_SNAPSHOT_H

#include "dialog.h"
#include "ui_dialog_snapshot.h"

class DialogSnapshot : public Dialog, public Ui::DialogSnapshot
{
    Q_OBJECT

public:
    explicit DialogSnapshot(MainWindow *window);
    ~DialogSnapshot();

    void saveSnapshot(const QString& fileName);

private slots:
    void computeImageSize();
};

#endif // DIALOG_SNAPSHOT_H
