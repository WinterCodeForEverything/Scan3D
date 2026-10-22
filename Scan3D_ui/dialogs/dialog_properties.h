#ifndef DIALOG_PROPERTIES_H
#define DIALOG_PROPERTIES_H

#include "dialog.h"
#include "ui_dialog_properties.h"


namespace scan3d {
    class Model;
}

class DialogProperties : public Dialog, public Ui::DialogProperties {
Q_OBJECT

public:
    DialogProperties(MainWindow *window);
    ~DialogProperties();

private Q_SLOTS:

    void updateProperties();
    void applyCommand();

    void commandChanged(const QString &);
    void modelChanged(const QString &);
    void locationChanged(const QString &);
    void propertyChanged(const QString &);

private:
    scan3d::Model *getModel();

    bool removeProperty();
    bool renameProperty();
    bool convertPropertyDataType();
};

#endif
