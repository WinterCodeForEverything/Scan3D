#include "dialogs/dialog.h"

#include "main_window.h"

#include <QDockWidget>
#include <QStyle>
#include <QLayout>


Dialog::Dialog(MainWindow *window)
        : QDialog(window) {
    window_ = window;
    viewer_ = window_->viewer();

    const Qt::WindowFlags flags = windowFlags();
    setWindowFlags(flags | Qt::Tool);
}


Dialog::~Dialog() {

}
