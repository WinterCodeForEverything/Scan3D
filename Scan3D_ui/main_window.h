#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <string>

#include <QMainWindow>
#include <QProgressBar>
#include <QLabel>

#include <module/util/progress.h>
#include <module/core/types.h>


namespace Ui {
    class MainWindow;
}

namespace scan3d {
    class Model;
}

class PaintCanvas;
class WidgetPointsDrawable;
class WidgetLinesDrawable;
class WidgetTrianglesDrawable;

class MainWindow : public QMainWindow, public scan3d::ProgressClient
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    PaintCanvas* viewer() { return viewer_; }

    void setCurrentFile(const QString &fileName);

    void updateUi(); // entire ui: window tile, rendering panel, model panel
    void updateRenderingPanel();    // rendering panel only
	void updateStatusBar();    // status bar

    void setShowSelectedOnly(bool b);

    void showPointUnderMouse(const scan3d::vec3& p, bool found);

public slots:
    void enableCameraManipulation();

private slots:

    // file
    bool onOpen();
    bool onSave();
    void onOpenRecentFile();
    void onClearRecentFiles();

    // view
    void saveSnapshot();
    void setBackgroundColor();

    // point cloud
    void pointCloudDownsampling();
    void pointCloudEstimateNormals();
    void pointCloudReorientNormals();
	void pointCloudInvertNormalsOrientation();
    void pointCloudNormalizeNormals();
    void pointCloudPoissonSurfaceReconstruction();
	void pointCloudStatisticFilter();
	void pointCloudRadiusFilter();
	void pointCloudBilateralFilter();
	void pointCloudCurvalEstimate();
	void pointCloudComputeFeatureFPFH();
	void pointCloudRegistration();
	void pointCloudDelaunaySurfaceReconstruction();

    // surface mesh
	void surfaceMeshInvertFrontSurface();
	void surfaceMeshRemeshing();
	void surfaceMeshRemeshFixing();
	void surfaceMeshDetectSelfIntersections();
	void surfaceMeshRemeshSelfIntersections();
	
	void surfaceMeshTexturing();

    // user interaction
    void operationModeChanged(QAction* act);


    void cancelTask();

    // about
    void onAbout();
    void showManual();

protected:
    void dragEnterEvent(QDragEnterEvent *e) override;
    void dropEvent(QDropEvent *) override;
    void closeEvent(QCloseEvent *event) override;

private:
    // Open a file with file name given. On success, the model has been added to the viewer
    scan3d::Model* open(const std::string& file_name);

private:
    void createActionsForFileMenu();
    void createActionsForViewMenu();
    void createActionsForPointCloudMenu();
    void createActionsForSurfaceMeshMenu();

    void createStatusBar();

    bool okToContinue();
    void readSettings();
    void writeSettings();
    void updateWindowTitle();

    void updateRecentFileActions();
    QString strippedName(const QString &fullFileName);

    void notify(std::size_t value, bool show_text = true, bool update_viewer = true) override ;

private:
    PaintCanvas*   viewer_;

    QStringList recentFiles_;
    QString		curDataDirectory_;

    enum { MaxRecentFiles = 5 };
    QAction *actionsRecentFile[MaxRecentFiles],
        *actionSeparator;

    QProgressBar*	progress_bar_;
    QLabel *labelStatusInfo_,
            *labelPointUnderMouse_,
            *labelNumFaces_,
            *labelNumVertices_,
            *labelNumEdges_;

    WidgetPointsDrawable*       widgetPointsDrawable_;
    WidgetLinesDrawable*        widgetLinesDrawable_;
    WidgetTrianglesDrawable*    widgetTrianglesDrawable_;

private:
    Ui::MainWindow* ui;

};


#endif // MAIN_WINDOW_H
