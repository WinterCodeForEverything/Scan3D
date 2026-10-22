#include "main_window.h"

#include <string>
#include <iostream>
#include <stdexcept>

#include <QDir>
#include <QTime>
#include <QApplication>
#include <QSplashScreen>
#ifdef _WIN32
#include <QStyleFactory>
#endif
#include <QSurfaceFormat>
#include <QElapsedTimer>

#include <module/fileio/resources.h>
#include <module/util/logging.h>


using namespace scan3d;


// reimplemented from QApplication so we can throw exceptions in slots/event handler
class TDScan : public QApplication
{
public:
	TDScan(int& argc, char ** argv) : QApplication(argc, argv) { }
    virtual ~TDScan() { }
    virtual bool notify(QObject * receiver, QEvent * event) {
        try {
            return QApplication::notify(receiver, event);
        } catch(std::exception& e) {
            LOG(ERROR) << "an exception was thrown: " << e.what();
        }
        return false;
    }
};


int main(int argc, char *argv[])
{
    //Locale management
    {
        //Force 'English' locale so as to get a consistent behavior everywhere
        QLocale locale = QLocale(QLocale::English);
        locale.setNumberOptions(QLocale::c().numberOptions());
        QLocale::setDefault(locale);

#ifdef Q_OS_UNIX
        //We reset the numeric locale for POSIX functions
        //See http://qt-project.org/doc/qt-5/qcoreapplication.html#locale-settings
        setlocale(LC_NUMERIC, "C");
#endif
    }

    // Note: Calling QSurfaceFormat::setDefaultFormat() before constructing the
    //       QApplication instance is mandatory on some platforms(for example, macOS)
    //       when an OpenGL core profile context is requested. This is to ensure
    //       that resource sharing between contexts stays functional as all internal
    //       contexts are created using the correct version and profile.
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
#ifndef NDEBUG
    format.setOption(QSurfaceFormat::DebugContext);
#endif
    QSurfaceFormat::setDefaultFormat(format);

    // Commented to let Qt choose the most suitable OpenGL implementation
	//QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
	QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
#if (QT_VERSION >= QT_VERSION_CHECK(5, 7, 0))
    //QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
#endif

#ifdef __APPLE__
    //QApplication::setAttribute(Qt::AA_DontUseNativeMenuBar);
#endif

	TDScan app(argc, argv);
#ifdef _WIN32   // to have similar style as on macOS.
    app.setStyle(QStyleFactory::create("Fusion"));
#endif

     QDir workingDir = QCoreApplication::applicationDirPath();
#ifdef __APPLE__
    // This makes sure that our "working directory" is not within the application bundle
    if ( workingDir.dirName() == "MacOS" ) {
        workingDir.cdUp();
        workingDir.cdUp();
        workingDir.cdUp();
    }
#endif
    QDir::setCurrent(workingDir.absolutePath());

    logging::initialize(true, "", google::GLOG_INFO);

    try {
        MainWindow win;
        win.show();

#ifdef NDEBUG
        QApplication::processEvents();
#endif

        return app.exec();
    }
    catch (const std::exception& e) {
        const std::string msg =
                std::string("Oh sorry, 3DScan crashed.\n") +
                "Error message: " + e.what() + ".";
        LOG(ERROR) << msg;
    }

    return EXIT_FAILURE;
}
