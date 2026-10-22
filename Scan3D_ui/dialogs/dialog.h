#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

class MainWindow;
class PaintCanvas;


class Dialog : public QDialog {
Q_OBJECT

public:
    Dialog(MainWindow *window);
    ~Dialog();

protected:

    PaintCanvas *viewer_;
    MainWindow *window_;
};

#endif
