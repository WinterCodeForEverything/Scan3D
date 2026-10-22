#include "main_window.h"

#include <string>
#include <iostream>

#include <QFileDialog>
#include <QDropEvent>
#include <QMimeData>
#include <QSettings>
#include <QMessageBox>
#include <QColorDialog>
#include <QCoreApplication>
#include <QPushButton>

#include <module/core/surface_mesh.h>
#include <module/core/graph.h>
#include <module/core/point_cloud.h>
#include <module/core/random.h>
#include <module/renderer/camera.h>
#include <module/renderer/renderer.h>
#include <module/renderer/drawable_triangles.h>
#include <module/fileio/point_cloud_io.h>
#include <module/fileio/graph_io.h>
#include <module/fileio/surface_mesh_io.h>
#include <module/fileio/ply_reader_writer.h>
#include <module/fileio/obj_reader_judge.h>
#include <module/fileio/point_cloud_io_ptx.h>
#include <module/fileio/resources.h>
#include <module/algo/point_cloud_normals.h>
#include <module/algo/surface_mesh_texturing.h>
#include <module/algo/surface_mesh_fixing.h>
#include <module/algo_ext/mesh_surfacer.h>
#include <module/util/logging.h>
#include <module/util/file_system.h>
#include <module/util/stop_watch.h>

#include "paint_canvas.h"

#include "dialogs/dialog_snapshot.h"
#include "dialogs/dialog_properties.h"
#include "dialogs/dialog_poisson_reconstruction.h"
#include "dialogs/dialog_point_cloud_simplification.h"
#include "dialogs/dialog_point_cloud_statistic_filter.h"
#include "dialogs/dialog_point_cloud_radius_filter.h"
#include "dialogs/dialog_point_cloud_bilateral_filter.h"
#include "dialogs/dialog_point_cloud_curvation_estimate.h"
#include "dialogs/dialog_point_cloud_feature_FPFH.h"
#include "dialogs/dialog_point_cloud_registration.h"
#include "dialogs/dialog_point_cloud_delaunay_reconstruction.h"
#include "dialogs/dialog_surface_mesh_remeshing.h"
#include "dialogs/dialog_surface_mesh_texturing.h"

#include "widgets/widget_global_setting.h"
#include "widgets/widget_drawable_points.h"
#include "widgets/widget_drawable_lines.h"
#include "widgets/widget_drawable_triangles.h"

#include <ui_main_window.h>


using namespace scan3d;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->treeWidgetModels->init(this);

    viewer_ = new PaintCanvas(this);
    setCentralWidget(viewer_);

    // ----- the width of the rendering panel ------
    // sizeHint() doesn't suggest a good value
    // const QSize& size = ui->dockWidgetRendering->sizeHint();
    const int width = 270; //static_cast<int>(size.width() * 1.5f);
    ui->dockWidgetRendering->setFixedWidth(width);

    // ----- rendering panel ------

    widgetTrianglesDrawable_ = new WidgetTrianglesDrawable(this);
    ui->verticalLayoutTrianglesDrawable->addWidget(widgetTrianglesDrawable_);
    widgetTrianglesDrawable_->setEnabled(false);

    widgetLinesDrawable_ = new WidgetLinesDrawable(this);
    ui->verticalLayoutLinesDrawable->addWidget(widgetLinesDrawable_);
    widgetLinesDrawable_->setEnabled(false);

    widgetPointsDrawable_ = new WidgetPointsDrawable(this);
    ui->verticalLayoutPointsDrawable->addWidget(widgetPointsDrawable_);
    widgetPointsDrawable_->setEnabled(false);

    ui->verticalLayoutGlobalSetting->addWidget(new WidgetGlobalSetting(this));


    // file menu
    createActionsForFileMenu();

    // view menu
    createActionsForViewMenu();

    // point cloud menu
    createActionsForPointCloudMenu();

    // surface mesh menu
    createActionsForSurfaceMeshMenu();

    // status bar
    createStatusBar();

    // about menu
    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(onAbout()));
    connect(ui->actionManual, SIGNAL(triggered()), this, SLOT(showManual()));

    // options for the model panel
    connect(ui->checkBoxAutoFocus, SIGNAL(toggled(bool)), ui->treeWidgetModels, SLOT(setAutoFocus(bool)));
    connect(ui->checkBoxSelectedOnly, SIGNAL(toggled(bool)), ui->treeWidgetModels, SLOT(setSelectedOnly(bool)));

    setWindowIcon(QIcon(QString::fromStdString(resource::directory() + "/icons/3DScan.png")));
    setContextMenuPolicy(Qt::CustomContextMenu);
    setAcceptDrops(true);

#ifdef NDEBUG
    setWindowState(Qt::WindowMaximized);
#else
    setBaseSize(1024, 800);
#endif

    readSettings();
    updateWindowTitle();
}


MainWindow::~MainWindow() {
    LOG(INFO) << "3DScan terminated. Bye!";
}


void MainWindow::notify(std::size_t value, bool show_text, bool update_viewer) {
    progress_bar_->setValue(int(value));
    progress_bar_->setTextVisible(show_text);
    progress_bar_->setVisible(value > 0 && value < 100);

    if (update_viewer) {
        viewer_->update();
        // This approach has significant drawbacks. For example, imagine you wanted to perform two such loops
        // in parallel-calling one of them would effectively halt the other until the first one is finished
        // (so you can't distribute computing power among different tasks). It also makes the application react
        // with delays to events. Furthermore the code is difficult to read and analyze, therefore this solution
        // is only suited for short and simple problems that are to be processed in a single thread, such as
        // splash screens and the monitoring of short operations.
        QCoreApplication::processEvents();
    }
}


void MainWindow::createStatusBar()
{
    labelStatusInfo_ = new QLabel("Ready");
    labelStatusInfo_->setFixedWidth(ui->dockWidgetRendering->width());
    labelStatusInfo_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(labelStatusInfo_);

    labelPointUnderMouse_ = new QLabel("XYZ = [-, -, -]");
    labelPointUnderMouse_->setFixedWidth(300);
    labelPointUnderMouse_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(labelPointUnderMouse_);

    //////////////////////////////////////////////////////////////////////////

    const int length = 150;
    labelNumFaces_ = new QLabel;
    labelNumFaces_->setMinimumWidth(length);
    labelNumFaces_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(labelNumFaces_);

    labelNumVertices_ = new QLabel;
    labelNumVertices_->setMinimumWidth(length);
    labelNumVertices_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(labelNumVertices_);

    labelNumEdges_ = new QLabel;
    labelNumEdges_->setMinimumWidth(length);
    labelNumEdges_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(labelNumEdges_);

    //////////////////////////////////////////////////////////////////////////

    progress_bar_ = new QProgressBar;
    progress_bar_->setVisible(false);
    progress_bar_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    progress_bar_->setMinimumWidth(ui->dockWidgetModels->sizeHint().width());
    statusBar()->addPermanentWidget(progress_bar_);

    //////////////////////////////////////////////////////////////////////////

    updateStatusBar();
}


void MainWindow::updateStatusBar()
{
    QString faces(""), vertices(""), edges("");

    Model* model = viewer_->currentModel();
    if (dynamic_cast<SurfaceMesh*>(model)) {
        auto mesh = dynamic_cast<SurfaceMesh*>(model);
        faces = QString("#faces: %1").arg(mesh->n_faces());
        vertices = QString("#vertices: %1").arg(mesh->n_vertices());
        edges = QString("#edges: %1").arg(mesh->n_edges());
        labelNumFaces_->setVisible(true);
        labelNumEdges_->setVisible(true);
    }

    else if (dynamic_cast<PointCloud*>(model)) {
        auto cloud = dynamic_cast<PointCloud*>(model);
        vertices = QString("#vertices: %1").arg(cloud->n_vertices());
        labelNumFaces_->setVisible(false);
        labelNumEdges_->setVisible(false);
    }

    else if (dynamic_cast<Graph*>(model)) {
        auto graph = dynamic_cast<Graph*>(model);
        vertices = QString("#vertices: %1").arg(graph->n_vertices());
        edges = QString("#meta: %1").arg(graph->n_edges());
        labelNumFaces_->setVisible(false);
        labelNumEdges_->setVisible(true);
    }

    labelNumFaces_->setText( faces );
    labelNumVertices_->setText( vertices );
    labelNumEdges_->setText( edges );
}


void MainWindow::cancelTask() {
    int value = progress_bar_->value() ;

    cancel();
    progress_bar_->reset();
    progress_bar_->setTextVisible(false);
    viewer_->update();

    if (value != -1 && value != 0)
        LOG(WARNING) << "task canceled" << std::endl;
}


void MainWindow::dragEnterEvent(QDragEnterEvent *e) {
    if (e->mimeData()->hasUrls())
        e->acceptProposedAction();
}


void MainWindow::dropEvent(QDropEvent *e) {
    if (e->mimeData()->hasUrls())
        e->acceptProposedAction();

    int count = 0;
    foreach (const QUrl &url, e->mimeData()->urls()) {
        const QString &fileName = url.toLocalFile();
        if (open(fileName.toStdString()))
            ++count;
    }

    if (count > 0)
        viewer_->update();
}


bool MainWindow::onOpen() {
    const QStringList& fileNames = QFileDialog::getOpenFileNames(
                this,
                "Open file(s)",
                curDataDirectory_,
                "Supported formats (*.ply *.obj *.off *.stl *.poly *.bin *.las *.laz *.xyz *.asc *.bxyz *.vg *.bvg *.ptx *.3mf)\n"
                "Mesh formats (*.ply *.obj *.off *.stl *.poly *.3mf)\n"
                "Point set formats (*.ply *.obj *.bin *.ptx *.las *.laz *.xyz *.asc *.bxyz *.vg *.bvg *.ptx)\n"
                "All formats (*.*)"
            );

    // Hide closed dialog
    QApplication::processEvents();

    if (fileNames.empty())
        return false;

    int count = 0;
	ProgressLogger progress(fileNames.size());
    for (const auto& name : fileNames) {
        if (progress.is_canceled())
            break;
        if (open(name.toStdString()))
            ++count;
        progress.next();
    }
    if (count > 0)
        viewer()->fitScreen();

    return count > 0;
}


bool MainWindow::onSave() {
    const Model* model = viewer_->currentModel();
    if (!model) {
        std::cerr << "no model exists" << std::endl;
        return false;
    }

    std::string default_file_name = model->name();
    if (file_system::extension(default_file_name).empty()) // no extension?
        default_file_name += ".ply"; // default to ply

    const QString& fileName = QFileDialog::getSaveFileName(
                this,
                "Open file(s)",
                QString::fromStdString(default_file_name),
                "Supported formats (*.ply *.obj *.off *.stl *.poly *.bin *.las *.laz *.xyz *.asc *.bxyz *.vg *.bvg)\n"
                "Mesh formats (*.ply *.obj *.off *.stl *.poly *.3mf)\n"
                "Point set formats (*.ply *.obj *.bin *.ptx *.las *.laz *.xyz *.bxyz *.vg *.bvg)\n"
                "All formats (*.*)"
    );

    if (fileName.isEmpty())
        return false;

    bool saved = false;
    if (dynamic_cast<const PointCloud*>(model)) {
        const PointCloud* cloud = dynamic_cast<const PointCloud*>(model);
        saved = PointCloudIO::save(fileName.toStdString(), cloud);
    }
    else if (dynamic_cast<const SurfaceMesh*>(model)) {
        const SurfaceMesh* mesh = dynamic_cast<const SurfaceMesh*>(model);
        saved = SurfaceMeshIO::save(fileName.toStdString(), mesh);
    }
    else if (dynamic_cast<const Graph*>(model)) {
        const Graph* graph = dynamic_cast<const Graph*>(model);
        saved = GraphIO::save(fileName.toStdString(), graph);
    }

    if (saved) {
        LOG(INFO) << "model successfully saved to: " << fileName.toStdString();
        setCurrentFile(fileName);
        return true;
    }

    return false;
}


Model* MainWindow::open(const std::string& file_name) {
    auto models = viewer_->models();
    for (auto m : models) {
        if (m->name() == file_name) {
            LOG(WARNING) << "model alreaded loaded: " << file_name;
            return nullptr;
        }
    }

    const std::string& ext = file_system::extension(file_name, true);
    bool is_ply_mesh = false;
    if (ext == "ply")
        is_ply_mesh = (io::PlyReader::num_instances(file_name, "face") > 0);

	bool is_obj_mesh = false;
	if (ext == "obj")
	{
		io::ObjReaderJudge obj_reader_judge = io::ObjReaderJudge(file_name);
		if (!obj_reader_judge.obj_file_is_valid())
			return nullptr;
		if( obj_reader_judge.obj_is_mesh() )
			is_obj_mesh = true;
	}

    Model* model = nullptr;
    if ((ext == "ply" && is_ply_mesh) || (ext == "obj" && is_obj_mesh) || ext == "off" || ext == "stl" || ext == "poly" || ext == "plg" || ext == "3mf") { // mesh
        model = SurfaceMeshIO::load(file_name);
    }
    else if (ext == "ply" && io::PlyReader::num_instances(file_name, "edge") > 0) {
        model = GraphIO::load(file_name);
    }
    else { // point cloud
        if (ext == "ptx") {
            io::PointCloudIO_ptx serializer(file_name);
            PointCloud* cloud = nullptr;
            while ((cloud = serializer.load_next())) {
                viewer_->addModel(cloud);
                ui->treeWidgetModels->addModel(cloud, true);
            }
            viewer_->fitScreen();
        }
		else
			model = PointCloudIO::load(file_name);
    }

    if (model) {
        model->set_name(file_name);
        viewer_->addModel(model);
        viewer_->fitScreen(model);
        ui->treeWidgetModels->addModel(model, true);
    }

    return model;
}


void MainWindow::updateUi() {
    const Model* model = viewer_->currentModel();
    if (model) {
        const std::string& name = model->name();
        setCurrentFile(QString::fromStdString(name));
    } else
        updateWindowTitle();

    ui->treeWidgetModels->updateModelList();
    updateRenderingPanel();
    updateStatusBar();
}


void MainWindow::updateRenderingPanel() {
    widgetTrianglesDrawable_->updatePanel();
    widgetLinesDrawable_->updatePanel();
    widgetPointsDrawable_->updatePanel();
}


void MainWindow::setCurrentFile(const QString &fileName)
{
    QString dir = fileName.left(fileName.lastIndexOf("/"));
    if (!dir.isEmpty() && file_system::is_directory(dir.toStdString()))
        curDataDirectory_ = dir;

    setWindowModified(false);

    if (!fileName.isEmpty()) {
        recentFiles_.removeAll(fileName);
        recentFiles_.prepend(fileName);
        updateRecentFileActions();
    }

    updateWindowTitle();
}


void MainWindow::enableCameraManipulation() {
    ui->actionCameraManipulation->trigger();
}


void MainWindow::setShowSelectedOnly(bool b) {
    ui->checkBoxSelectedOnly->setChecked(b);
}


void MainWindow::showPointUnderMouse(const scan3d::vec3 &p, bool found) {
	QString coordString = "XYZ = [-, -, -]";
	if (found)
		coordString = QString("XYZ = [%1, %2, %3]").arg(p.x).arg(p.y).arg(p.z);
	labelPointUnderMouse_->setText(coordString);
}


void MainWindow::onOpenRecentFile()
{
    if (okToContinue()) {
        QAction *action = qobject_cast<QAction *>(sender());
        if (action) {
            const QString filename(action->data().toString());
            if (open(filename.toStdString()))
                viewer_->update();
        }
    }
}


void MainWindow::onClearRecentFiles() {
    recentFiles_.clear();
    updateRecentFileActions();
}


void MainWindow::saveSnapshot() {
    const Model* model = viewer_->currentModel();

    const bool overwrite = false;
    std::string default_file_name("untitled.png");
    if (model)
        default_file_name = file_system::replace_extension(model->name(), "png");

    QString proposedFormat = "PNG (*.png)";
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Choose an image file name",
        QString::fromStdString(default_file_name),
        "Image formats (*.png *.jpg *.bmp *.ppm)\n"
        "PNG (*.png)\n"
        "JPG (*.jpg)\n"
        "Windows Bitmap (*.bmp)\n"
        "24bit RGB Bitmap (*.ppm)",
        &proposedFormat,
        overwrite ? QFileDialog::DontConfirmOverwrite : QFlags<QFileDialog::Option>(nullptr)
    );

    // Hide closed dialog
    QApplication::processEvents();

    if (fileName.isEmpty())
        return;

    DialogSnapshot dialog(this);
    if (dialog.exec() == QDialog::Accepted)
        dialog.saveSnapshot(fileName);
}


void MainWindow::setBackgroundColor() {
    const vec4& c = viewer_->backGroundColor();
    QColor orig(static_cast<int>(c.r * 255), static_cast<int>(c.g * 255), static_cast<int>(c.b * 255), static_cast<int>(c.a * 255));
    const QColor& color = QColorDialog::getColor(orig, this);
    if (color.isValid()) {
        const vec4 newColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
        viewer_->setBackgroundColor(newColor);
        viewer_->update();
    }
}


bool MainWindow::okToContinue()
{
    if (isWindowModified()) {
        int r = QMessageBox::warning(this, tr("3DScan"),
            tr("The model has been modified.\n"
            "Do you want to save your changes?"),
            QMessageBox::Yes | QMessageBox::Default,
            QMessageBox::No,
            QMessageBox::Cancel | QMessageBox::Escape);
        if (r == QMessageBox::Yes)
            return onSave();
        else if (r == QMessageBox::Cancel)
            return false;
    }
    return true;
}


void MainWindow::onAbout()
{
    QString title = QMessageBox::tr(
        "<p align=\"center\"><span style=\"font-style:italic;\">I'm good software, though I have defects.</span></p>"
        );

#if defined (ENV_32_BIT)
    title += QMessageBox::tr("<h3>3DScan (32-bit)</h3>");
#elif defined (ENV_64_BIT)
    title += QMessageBox::tr("<h3>3DScan (64-bit)</h3>");
#else
    title += QMessageBox::tr("<h3>3DScan</h3>");
#endif

#ifndef NDEBUG
    title += QMessageBox::tr(" (Debug Version)");
#endif

    QString text = QMessageBox::tr(
        "<p><h4> Build %1</h4></p>"
        "<p>3DScan is software for processing and rendering meshes and point clouds.</p>"
        "<p>Jianwei Xu<br>"
        "<a href=\"mailto: jianwei.xu@gmail.com\"> jianwei.xu@gmail.com</a><br>"
        ).arg("20181206");

    //QMessageBox::about(this, title, text);
    QMessageBox::about(this, "About 3DScan", title + text);
}


void MainWindow::showManual() {
    std::cout << viewer()->usage() << std::endl;
}


void MainWindow::readSettings()
{
    QSettings settings(" jianwei.xu@gmail.com", "3DScan");
    recentFiles_ = settings.value("recentFiles").toStringList();
    updateRecentFileActions();
    curDataDirectory_ = settings.value("currentDirectory").toString();
}


void MainWindow::writeSettings()
{
    QSettings settings(" jianwei.xu@gmail.com", "3DScan");
    settings.setValue("recentFiles", recentFiles_);
    if (!curDataDirectory_.isEmpty() && file_system::is_directory(curDataDirectory_.toStdString()))
        settings.setValue("currentDirectory", curDataDirectory_);
}


void MainWindow::updateWindowTitle() {
    Model* model = viewer_->currentModel();

#ifndef NDEBUG
    QString title = "3DScan (Debug Version)";
#else
    QString title = "3DScan";
#endif // _DEBUG

    QString fileName("Untitled");
    if (model)
        fileName = QString::fromStdString(model->name());

    title = tr("%1[*] - %2").arg(strippedName(fileName)).arg(title);
    setWindowTitle(title);
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    if (okToContinue()) {
        writeSettings();
        event->accept();
    } else {
        event->ignore();
    }
}


void MainWindow::updateRecentFileActions()
{
    QMutableStringListIterator i(recentFiles_);
    while (i.hasNext()) {
        if (!QFile::exists(i.next()))
            i.remove();
    }

    for (int j = 0; j < MaxRecentFiles; ++j) {
        if (j < recentFiles_.count()) {
            QString text = tr("&%1 %2").arg(j + 1).arg(strippedName(recentFiles_[j]));
            actionsRecentFile[j]->setText(text);
            actionsRecentFile[j]->setData(recentFiles_[j]);
            actionsRecentFile[j]->setVisible(true);
        } else {
            actionsRecentFile[j]->setVisible(false);
        }
    }

    actionSeparator->setVisible(!recentFiles_.isEmpty());
}


QString MainWindow::strippedName(const QString &fullFileName)
{
    return QFileInfo(fullFileName).fileName();
}


void MainWindow::createActionsForFileMenu() {
    connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(onSave()));

    actionSeparator = ui->menuFile->addSeparator();

    QList<QAction*> actions;
    for (int i = 0; i < MaxRecentFiles; ++i) {
        actionsRecentFile[i] = new QAction(this);
        actionsRecentFile[i]->setVisible(false);
        connect(actionsRecentFile[i], SIGNAL(triggered()), this, SLOT(onOpenRecentFile()));

        actions.push_back(actionsRecentFile[i]);
    }
    ui->menuRecentFiles->insertActions(ui->actionClearRecentFiles, actions);
    ui->menuRecentFiles->insertSeparator(ui->actionClearRecentFiles);
    connect(ui->actionClearRecentFiles, SIGNAL(triggered()), this, SLOT(onClearRecentFiles()));

    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
    ui->actionExit->setShortcut(QString("Ctrl+Q"));
}


void MainWindow::createActionsForViewMenu() {

	ui->menuView->hide();
    connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(saveSnapshot()));

    ui->menuView->addSeparator();

    connect(ui->actionCopyCamera, SIGNAL(triggered()), viewer_, SLOT(copyCamera()));
    connect(ui->actionPasteCamera, SIGNAL(triggered()), viewer_, SLOT(pasteCamera()));

    QAction* actionToggleDockWidgetRendering = ui->dockWidgetRendering->toggleViewAction();
    actionToggleDockWidgetRendering->setText("Rendering Panel");
    ui->menuView->addAction(actionToggleDockWidgetRendering);

    QAction* actionToggleDockWidgetModels = ui->dockWidgetModels->toggleViewAction();
    actionToggleDockWidgetModels->setText("Model Panel");
    ui->menuView->addAction(actionToggleDockWidgetModels);

    connect(ui->actionBackgroundColor, SIGNAL(triggered()), this, SLOT(setBackgroundColor()));
}


void MainWindow::createActionsForPointCloudMenu() {
    connect(ui->actionDownSampling, SIGNAL(triggered()), this, SLOT(pointCloudDownsampling()));

    connect(ui->actionEstimatePointCloudNormals, SIGNAL(triggered()), this, SLOT(pointCloudEstimateNormals()));
    connect(ui->actionReorientPointCloudNormals, SIGNAL(triggered()), this, SLOT(pointCloudReorientNormals()));
	connect(ui->actionInvertPointCloudNormalsNrientation, SIGNAL(triggered()), this, SLOT(pointCloudInvertNormalsOrientation()));
    connect(ui->actionNormalizePointCloudNormals, SIGNAL(triggered()), this, SLOT(pointCloudNormalizeNormals()));

    //connect(ui->actionRansacPrimitiveExtraction, SIGNAL(triggered()), this, SLOT(pointCloudRansacPrimitiveExtraction()));
    connect(ui->actionPoissonSurfaceReconstruction, SIGNAL(triggered()), this, SLOT(pointCloudPoissonSurfaceReconstruction()));
	//
    //connect(ui->actionDelaunayTriangulation2D, SIGNAL(triggered()), this, SLOT(pointCloudDelaunayTriangulation2D()));
    //connect(ui->actionDelaynayTriangulation3D, SIGNAL(triggered()), this, SLOT(pointCloudDelaunayTriangulation3D()));

	connect(ui->actionPointCloudStatisticFilter, SIGNAL(triggered()), this, SLOT(pointCloudStatisticFilter()));
	connect(ui->actionPointCloudRadiusFilter, SIGNAL(triggered()), this, SLOT(pointCloudRadiusFilter()));
	connect(ui->actionPointCloudBilateralFilter, SIGNAL(triggered()), this, SLOT(pointCloudBilateralFilter()));

	connect(ui->actionPointCloudCurvalEstimate, SIGNAL(triggered()), this, SLOT(pointCloudCurvalEstimate()));

	connect(ui->actionPointCloudComputeFeatureFPFH, SIGNAL(triggered()), this, SLOT(pointCloudComputeFeatureFPFH()));
	connect(ui->actionPointCloudRegistration, SIGNAL(triggered()), this, SLOT(pointCloudRegistration()));

	connect(ui->actionDelaynaySurfaceReconstruction, SIGNAL(triggered()), this, SLOT(pointCloudDelaunaySurfaceReconstruction()));
}


void MainWindow::createActionsForSurfaceMeshMenu() {
	connect(ui->actionSurfaceMeshInvertFrontSurface, SIGNAL(triggered()), this, SLOT(surfaceMeshInvertFrontSurface()));
	connect(ui->actionSurface_Mesh_Remeshing, SIGNAL(triggered()), this, SLOT(surfaceMeshRemeshing()));
	connect(ui->actionSurface_Mesh_Remesh_Fixing, SIGNAL(triggered()), this, SLOT(surfaceMeshRemeshFixing()));
	connect(ui->actionSurface_Mesh_DetectSelfIntersections, SIGNAL(triggered()), this, SLOT(surfaceMeshDetectSelfIntersections()));
	connect(ui->actionSurface_Mesh_RemeshSelfIntersections, SIGNAL(triggered()), this, SLOT(surfaceMeshRemeshSelfIntersections()));
	connect(ui->actionSurface_Mesh_Texturing, SIGNAL(triggered()), this, SLOT(surfaceMeshTexturing()));
}


void MainWindow::operationModeChanged(QAction* act) {
    if (act == ui->actionCameraManipulation) {
        viewer()->tool_manager()->set_tool(tools::ToolManager::EMPTY_TOOL);
    }
    else if (act == ui->actionSelectClick) {
        if (dynamic_cast<SurfaceMesh*>(viewer()->currentModel()))
            viewer()->tool_manager()->set_tool(tools::ToolManager::SELECT_SURFACE_MESH_FACE_CLICK_TOOL);
    }
    else if (act == ui->actionSelectRect) {
        if (dynamic_cast<SurfaceMesh*>(viewer()->currentModel()))
            viewer()->tool_manager()->set_tool(tools::ToolManager::SELECT_SURFACE_MESH_FACE_RECT_TOOL);
        else if (dynamic_cast<PointCloud*>(viewer()->currentModel()))
            viewer()->tool_manager()->set_tool(tools::ToolManager::SELECT_POINT_CLOUD_RECT_TOOL);
    }
    else if (act == ui->actionSelectLasso) {
        if (dynamic_cast<SurfaceMesh*>(viewer()->currentModel()))
            viewer()->tool_manager()->set_tool(tools::ToolManager::SELECT_SURFACE_MESH_FACE_LASSO_TOOL);
        else if (dynamic_cast<PointCloud*>(viewer()->currentModel()))
            viewer()->tool_manager()->set_tool(tools::ToolManager::SELECT_POINT_CLOUD_LASSO_TOOL);
    }
    viewer()->update();
}

void MainWindow::surfaceMeshInvertFrontSurface()
{
	SurfaceMesh* mesh = dynamic_cast<SurfaceMesh*>(viewer()->currentModel());
	if (!mesh)
		return;
	std::cout << "it may lose the porperties of the surface mesh!" << std::endl;
	SurfaceMesh* invertMesh = new SurfaceMesh;
	std::vector<SurfaceMesh::Vertex> all_vertices;
	for (auto p : mesh->points())
	{
		auto vv = invertMesh->add_vertex(p);
		all_vertices.push_back(vv);
	}
		
	for (auto f = mesh->faces_begin(); f != mesh->faces_end(); ++f)
	{
		std::vector<SurfaceMesh::Vertex> face_vts;
		auto vertices = mesh->vertices(*f);
		for (auto v = vertices.begin(); v != vertices.end(); ++v)
		{
			face_vts.push_back(all_vertices[(*v).idx()]);
		}
		std::reverse(face_vts.begin(), face_vts.end() );
		invertMesh->add_face(face_vts);
	}
	invertMesh->set_name(mesh->name());
	viewer()->deleteModel(mesh);
	viewer()->addModel(invertMesh);
	viewer()->setCurrentModel(invertMesh);
	viewer()->update();
	updateUi();
}

void MainWindow::surfaceMeshRemeshing() {
	static DialogSurfaceMeshRemeshing* dialog = nullptr;
	if (!dialog)
		dialog = new DialogSurfaceMeshRemeshing(this);
	dialog->show();
}

void MainWindow::surfaceMeshRemeshFixing() {

	SurfaceMesh* mesh = dynamic_cast<SurfaceMesh*>(viewer()->currentModel());
	if (!mesh)
		return;
	SurfaceMeshFixing smf;
	SurfaceMesh* newMesh = smf.removeSelfInteraction( mesh );
	newMesh->set_name(mesh->name());
	viewer()->deleteModel(mesh);
	viewer()->addModel(newMesh);
	viewer()->setCurrentModel(newMesh);
	viewer_->update();
	updateUi();
}


void MainWindow::surfaceMeshDetectSelfIntersections() {
	SurfaceMesh* mesh = dynamic_cast<SurfaceMesh*>(viewer()->currentModel());
	if (!mesh)
		return;

#if HAS_CGAL
	StopWatch w;
	w.start();
	LOG(INFO) << "detecting intersecting faces...";

	MeshSurfacer ms;
	const auto& pairs = ms.detect_self_intersections(mesh);
	if (!pairs.empty())
		LOG(INFO) << "done. " << pairs.size() << " pairs of faces intersect. " << w.time_string();
	else
		LOG(INFO) << "done. No intersecting faces detected. " << w.time_string();
#else
	LOG(WARNING) << "This function requires CGAL but CGAL was not found.";
#endif
}


void MainWindow::surfaceMeshRemeshSelfIntersections() {
	SurfaceMesh* mesh = dynamic_cast<SurfaceMesh*>(viewer()->currentModel());
	if (!mesh)
		return;

#if HAS_CGAL
	StopWatch w;
	w.start();
	LOG(INFO) << "remeshing intersecting faces...";

	auto size = mesh->n_faces();
	MeshSurfacer ms;
	SurfaceMesh* result = ms.remesh_self_intersections(mesh, true);
	if (result) {
		const std::string& name = file_system::name_less_extension(mesh->name()) + "_remeshed." + file_system::extension(mesh->name());
		result->set_name(name);
		viewer()->addModel(result);
		LOG(INFO) << "done. #faces " << size << " -> " << result->n_faces() << ". " << w.time_string();
		mesh->renderer()->update();
		viewer_->update();
		updateUi();
	}
	else
		LOG(INFO) << "done. No intersecting faces detected. " << w.time_string();
#else
	LOG(WARNING) << "This function requires CGAL but CGAL was not found.";
#endif
}


void MainWindow::surfaceMeshTexturing() {
	static DialogSurfaceMeshTexturing* dialog = nullptr;
	if (!dialog)
		dialog = new DialogSurfaceMeshTexturing(this, curDataDirectory_);
	dialog->show();
}


void MainWindow::pointCloudEstimateNormals() {
    PointCloud* cloud = dynamic_cast<PointCloud*>(viewer()->currentModel());
    if (!cloud)
        return;

    PointCloudNormals pcn;
    std::cout << "show the parameter dialog" << std::endl;
    pcn.estimate(cloud);

    cloud->renderer()->update();
    viewer()->update();
}


void MainWindow::pointCloudReorientNormals() {
    PointCloud* cloud = dynamic_cast<PointCloud*>(viewer()->currentModel());
    if (!cloud)
        return;

    PointCloudNormals pcn;
    std::cout << "show the parameter dialog" << std::endl;
    pcn.reorient(cloud);

    cloud->renderer()->update();
    viewer()->update();
}

void MainWindow::pointCloudInvertNormalsOrientation()
{
	PointCloud* cloud = dynamic_cast<PointCloud*>(viewer()->currentModel());
	if (!cloud)
		return;

	PointCloudNormals pcn;
	pcn.invertOrientation(cloud);

	cloud->renderer()->update();
	viewer()->update();
}


void MainWindow::pointCloudNormalizeNormals() {
    PointCloud* cloud = dynamic_cast<PointCloud*>(viewer()->currentModel());
    if (!cloud)
        return;

    auto prop = cloud->get_vertex_property<vec3>("v:normal");
    if (!prop) {
        LOG(WARNING) << "point cloud does not have normal information";
        return;
    }

    auto &normals = prop.vector();
    for (auto &n : normals)
        n.normalize();

    cloud->renderer()->update();
    viewer()->update();
}


void MainWindow::pointCloudPoissonSurfaceReconstruction() {
    static DialogPoissonReconstruction* dialog = nullptr;
    if (!dialog)
        dialog = new DialogPoissonReconstruction(this);
    dialog->show();
}

void MainWindow::pointCloudDownsampling() {
    static DialogPointCloudSimplification* dialog = nullptr;
    if (!dialog)
        dialog = new DialogPointCloudSimplification(this);
    dialog->show();
}

void MainWindow::pointCloudStatisticFilter() {

	static DialogPointCloudStatisticFilter* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudStatisticFilter(this);
	dialog->show();
}

void MainWindow::pointCloudRadiusFilter() {

	static DialogPointCloudRadiusFilter* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudRadiusFilter(this);
	dialog->show();
}
void MainWindow::pointCloudBilateralFilter() {

	static DialogPointCloudBilateralFilter* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudBilateralFilter(this);
	dialog->show();
}

void MainWindow::pointCloudCurvalEstimate() {

	static DialogPointCloudCurvationEstimate* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudCurvationEstimate(this);
	dialog->show();
}

void MainWindow::pointCloudComputeFeatureFPFH(){

	static DialogPointCloudComputeFeatureFPFH* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudComputeFeatureFPFH(this);
	dialog->show();
}

void MainWindow::pointCloudRegistration() {

	static DialogPointCloudRegistration* dialog = nullptr;
	if (!dialog)
		dialog = new DialogPointCloudRegistration(this, curDataDirectory_);
	dialog->updatePointCloudList();
	dialog->show();
}

void MainWindow::pointCloudDelaunaySurfaceReconstruction() {

	static DialogDelaunayReconstruction* dialog = nullptr;
	if (!dialog)
		dialog = new DialogDelaunayReconstruction(this);
	dialog->show();
}
