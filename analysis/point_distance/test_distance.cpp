#include "CloudForgeAnalyzer.h"
#include "Basic/ViewportPointPicker.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QElapsedTimer>
#include <QLabel>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QToolBar>
#include <QThread>
#include <vtkBillboardTextActor3D.h>
#include <vtkCellArray.h>
#include <vtkDataSet.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPropCollection.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <cstdlib>

static int passed = 0, failed = 0;
static QString capturePath(const char* name)
{
    const char* suffix = std::getenv("DISTANCE_CAPTURE_SUFFIX");
    return QString("analysis/point_distance/%1%2.png").arg(name, suffix ? suffix : "");
}
static void check(bool ok, const char* name)
{
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    ok ? ++passed : ++failed;
}

static void pump(int ms = 80)
{
    QElapsedTimer timer;
    timer.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (timer.elapsed() < ms);
}

static void mouse(QWidget* view, QEvent::Type type, QPoint point, Qt::MouseButton button = Qt::NoButton,
                  Qt::MouseButtons buttons = Qt::NoButton)
{
    QMouseEvent event(type, QPointF(point), QPointF(view->mapToGlobal(point)), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}

static void click(QWidget* view, QPoint point)
{
    mouse(view, QEvent::MouseButtonPress, point, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, point, Qt::LeftButton);
    pump();
}

static QPoint screen(QWidget* view, vtkRenderer* renderer, const double* point)
{
    renderer->SetWorldPoint(point[0], point[1], point[2], 1.0);
    renderer->WorldToDisplay();
    const double* display = renderer->GetDisplayPoint();
    const int* size = renderer->GetRenderWindow()->GetSize();
    return QPoint(qRound(display[0] * view->width() / size[0]),
                  qRound((size[1] - 1 - display[1]) * view->height() / size[1]));
}

static vtkSmartPointer<vtkActor> cloudActor(const std::vector<MeasureDistance::Point>& data)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    auto cells = vtkSmartPointer<vtkCellArray>::New();
    for (const auto& point : data) {
        vtkIdType id = points->InsertNextPoint(point.data());
        cells->InsertNextCell(1, &id);
    }
    auto cloud = vtkSmartPointer<vtkPolyData>::New();
    cloud->SetPoints(points);
    cloud->SetVerts(cells);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(cloud);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(7);
    return actor;
}

static void mathTests()
{
    using M = MeasureDistance;
    auto r = M::compute({0, 0, 0}, {3, 4, 12});
    check(r.success && r.distance == 13.0, "3D 3-4-12 => 13");
    check(M::compute({5, -2, 7}, {5, -2, 7}).distance == 0.0, "same point => zero");
    check(M::compute({-3, -4, -12}, {0, 0, 0}).distance == 13.0, "negative coordinates");
    check(M::compute({1e10, 1e10, 1e10}, {1e10 + 3, 1e10 + 4, 1e10 + 12}).distance == 13.0, "translation invariance / double coordinates");
    check(M::compute({3, 4, 12}, {0, 0, 0}).distance == r.distance, "endpoint symmetry");
    auto large = M::compute({0, 0, 0}, {1e200, 1e200, 1e200});
    check(large.success && std::abs(large.distance / 1e200 - std::sqrt(3.0)) < 1e-14, "stable norm without square overflow");
    check(!M::compute({NAN, 0, 0}, {0, 0, 0}).success, "reject NaN");
    check(!M::compute({0, 0, 0}, {INFINITY, 0, 0}).success, "reject infinity");
    check(!M::compute({-1e308, 0, 0}, {1e308, 0, 0}).success, "reject unrepresentable difference");
}

struct StartupCamera {
    std::array<double, 13> values{};
    bool createdBeforeTool = false;
    bool createdAfterTool = false;
};

static StartupCamera startupScene(bool withTool)
{
    QVTKOpenGLNativeWidget view;
    view.resize(1100, 760);
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);
    auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>(renderer, window, "startup", false);
    view.setRenderWindow(window);
    QApplication::processEvents();
    view.makeCurrent();
    viewer->setupInteractor(view.interactor(), window);
    viewer->setBackgroundColor(0, 0.3, 0.4);
    viewer->addCoordinateSystem(4.0);
    StartupCamera snapshot;
    snapshot.createdBeforeTool = renderer->IsActiveCameraCreated();
    std::unique_ptr<ViewportPointPicker> tool;
    if (withTool) tool = std::make_unique<ViewportPointPicker>(&view, renderer, window);
    snapshot.createdAfterTool = renderer->IsActiveCameraCreated();
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile("PCDfiles/rabbit.pcd", *cloud);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "example");
    window->Render();
    view.show();
    pump(200);
    auto camera = renderer->GetActiveCamera();
    std::copy_n(camera->GetPosition(), 3, snapshot.values.begin());
    std::copy_n(camera->GetFocalPoint(), 3, snapshot.values.begin() + 3);
    std::copy_n(camera->GetViewUp(), 3, snapshot.values.begin() + 6);
    std::copy_n(camera->GetClippingRange(), 2, snapshot.values.begin() + 9);
    snapshot.values[11] = camera->GetParallelScale();
    snapshot.values[12] = camera->GetViewAngle();
    std::cout << "[STARTUP_CAMERA] tool=" << withTool << " created=" << snapshot.createdBeforeTool << "->"
              << snapshot.createdAfterTool << " position=" << snapshot.values[0] << ',' << snapshot.values[1]
              << ',' << snapshot.values[2] << " focal=" << snapshot.values[3] << ',' << snapshot.values[4]
              << ',' << snapshot.values[5] << std::endl;
    view.grab().save(capturePath(withTool ? "startup_with_tool" : "startup_baseline"));
    tool.reset();
    return snapshot;
}

static void startupCameraTests()
{
    auto baseline = startupScene(false);
    auto withTool = startupScene(true);
    check(withTool.createdBeforeTool == withTool.createdAfterTool, "tool construction preserves camera initialization timing");
    bool same = true;
    for (size_t i = 0; i < baseline.values.size(); ++i) {
        if (std::abs(baseline.values[i] - withTool.values[i]) > 1e-12) same = false;
    }
    check(same, "startup camera matches the original renderer without measurement tool");
}

static void guiTests()
{
    QVTKOpenGLNativeWidget view;
    view.resize(1100, 760);
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);
    view.setRenderWindow(window);
    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    view.interactor()->SetInteractorStyle(style);
    renderer->SetBackground(0.0, 0.3, 0.4);
    const MeasureDistance::Point a{0, 0, 0}, b{30, 40, 120};
    auto first = cloudActor({a});
    auto second = cloudActor({{0, 0, 0}});
    second->SetPosition(b.data()[0], b.data()[1], b.data()[2]);
    renderer->AddActor(first);
    renderer->AddActor(second);
    renderer->GetActiveCamera()->SetPosition(200, 200, 500);
    renderer->GetActiveCamera()->SetFocalPoint(15, 20, 60);
    renderer->ResetCamera();
    view.show();
    pump(400);
    std::cout << "[DISPLAY] viewport=" << view.width() << 'x' << view.height()
              << " render=" << window->GetSize()[0] << 'x' << window->GetSize()[1]
              << " dpi=" << window->GetDPI() << " dpr=" << view.devicePixelRatioF() << std::endl;
    ViewportPointPicker tool(&view, renderer, window);
    QString text;
    int completed = 0;
    QObject::connect(&tool, &ViewportPointPicker::resultChanged, [&](const QString& value) { text = value; });
    QObject::connect(&tool, &ViewportPointPicker::measurementCompleted, [&](const QString&) { ++completed; });
    check(!tool.start({}), "empty scene rejected");
    second->VisibilityOff();
    check(!tool.start({second}), "hidden clouds rejected");
    second->VisibilityOn();
    check(tool.start({first, second}), "activate tool with two cloud actors");
    QPoint pa = screen(&view, renderer, a.data()), pb = screen(&view, renderer, b.data());
    click(&view, pa);
    mouse(&view, QEvent::MouseMove, pb);
    pump();
    check(text.contains("130.000") && completed == 0, "hover previews actual world coordinates across transformed actors");
    MeasureDistance::Point cover;
    for (int i = 0; i < 3; ++i) cover[i] = b[i] + 0.1 * (renderer->GetActiveCamera()->GetPosition()[i] - b[i]);
    auto obstacle = cloudActor({cover});
    renderer->AddActor(obstacle);
    click(&view, pb);
    check(completed == 1 && text.contains("130.000"), "second click finalizes / measurement actors cannot be picked");
    window->Render();
    view.grab().save(capturePath("known_distance"));
    tool.stop();
    check(!tool.isActive() && text.contains("130.000"), "exit keeps completed result");
    tool.start({first, second});
    click(&view, pa);
    double cameraBefore[3];
    renderer->GetActiveCamera()->GetPosition(cameraBefore);
    mouse(&view, QEvent::MouseMove, pb);
    pump();
    mouse(&view, QEvent::MouseMove, QPoint(2, view.height() - 2));
    pump();
    check(text.isEmpty() && completed == 1, "blank hover removes stale preview");
    click(&view, QPoint(2, view.height() - 2));
    check(completed == 1, "blank click does not finalize");
    QKeyEvent esc(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QApplication::sendEvent(&view, &esc);
    check(!tool.isActive() && text.isEmpty(), "Esc cancels unfinished measurement");
    renderer->RemoveActor(obstacle);
    tool.start({first, second});
    click(&view, pa);
    mouse(&view, QEvent::MouseButtonPress, pb, Qt::LeftButton, Qt::LeftButton);
    mouse(&view, QEvent::MouseMove, pb + QPoint(50, 30), Qt::NoButton, Qt::LeftButton);
    mouse(&view, QEvent::MouseMove, pb, Qt::NoButton, Qt::LeftButton);
    mouse(&view, QEvent::MouseButtonRelease, pb, Qt::LeftButton);
    pump();
    check(completed == 1, "drag returning to origin does not select endpoint");
    const double* cameraAfter = renderer->GetActiveCamera()->GetPosition();
    check(std::hypot(cameraAfter[0] - cameraBefore[0], cameraAfter[1] - cameraBefore[1], cameraAfter[2] - cameraBefore[2]) > 0.01,
          "camera rotation stays available during selection");
    pb = screen(&view, renderer, b.data());
    click(&view, pb);
    check(completed == 2 && text.contains("130.000"), "distance invariant after rotating view");
    tool.clear();
    check(!tool.isActive() && text.isEmpty(), "clear removes result and exits");
    tool.start({first, second});
    pa = screen(&view, renderer, a.data());
    click(&view, pa);
    click(&view, pa);
    check(completed == 3 && text.contains("0.000 mm"), "coincident points finalize zero distance");
    view.grab().save(capturePath("zero_distance"));
    for (int i = 0; i < 20; ++i) { tool.stop(); tool.start({first, second}); }
    check(tool.isActive(), "repeat entry / exit without duplicate callbacks");
    tool.clear();

    std::vector<MeasureDistance::Point> large;
    large.reserve(2000000);
    for (int y = 0; y < 1000; ++y) for (int x = 0; x < 2000; ++x)
        large.push_back({x * 0.2, y * 0.2, 0.02 * std::sin(x * 0.03)});
    auto dense = cloudActor(large);
    dense->GetProperty()->SetPointSize(2);
    renderer->RemoveActor(first);
    renderer->RemoveActor(second);
    renderer->AddActor(dense);
    renderer->GetActiveCamera()->SetPosition(200, 100, 700);
    renderer->GetActiveCamera()->SetFocalPoint(200, 100, 0);
    renderer->ResetCamera();
    window->Render();
    tool.start({dense});
    const auto p0 = screen(&view, renderer, large[1000500].data());
    const auto p1 = screen(&view, renderer, large[1001500].data());
    click(&view, p0);
    QElapsedTimer timer;
    timer.start();
    for (int i = 0; i < 12; ++i) {
        mouse(&view, QEvent::MouseMove, p1 + QPoint(i, 0));
        pump(35);
    }
    std::cout << "[PERF] 2M points, 12 hover+render cycles: " << timer.elapsed() << " ms" << std::endl;
    check(!text.isEmpty(), "2M point cloud live preview");
    click(&view, p1);
    check(completed == 4, "2M point cloud endpoint commit");
    view.grab().save(capturePath("dense_cloud"));
    tool.clear();
    view.close();
}

static void mainWindowTests()
{
    CloudForgeAnalyzer main;
    main.resize(1450, 950);
    main.show();
    pump(500);
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    view->renderWindow()->Render();
    main.grab().save(capturePath("main_startup"));
    auto action = main.ui->measure_line;
    auto button = main.ui->mainToolBar->widgetForAction(action);
    check(button && button->isVisible() && action->text() == "两点测距", "sidebar button visible");
    click(button, button->rect().center());
    check(action->isChecked(), "actual sidebar click activates measurement");
    auto hint = view->findChild<QLabel*>("distanceMeasureHint");
    check(hint && hint->isVisible(), "operation hint visible in viewport");
    vtkActor* cloud = nullptr;
    auto props = renderer->GetViewProps();
    props->InitTraversal();
    while (auto prop = props->GetNextProp()) {
        auto actor = vtkActor::SafeDownCast(prop);
        if (actor && actor->GetPickable() && actor->GetMapper() && actor->GetMapper()->GetInput()
            && actor->GetMapper()->GetInput()->GetNumberOfPoints() > 100) cloud = actor;
    }
    check(cloud != nullptr, "sample cloud available for interaction");
    if (cloud) {
        auto data = cloud->GetMapper()->GetInput();
        double p[3], q[3];
        data->GetPoint(0, p);
        data->GetPoint(data->GetNumberOfPoints() / 2, q);
        click(view, screen(view, renderer, p));
        mouse(view, QEvent::MouseMove, screen(view, renderer, q));
        pump();
        click(view, screen(view, renderer, q));
        check(main.ui->textEdit_Show_CFmes->toPlainText().contains("空间距离:"), "main result panel receives measurement");
        main.grab().save(capturePath("main_window"));
    }
    main.ui->textEdit_Show_CFmes->setFocus();
    QKeyEvent panelEsc(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QApplication::sendEvent(main.ui->textEdit_Show_CFmes, &panelEsc);
    pump();
    check(!action->isChecked(), "Esc exits even when result panel has focus");
    click(button, button->rect().center());
    click(button, button->rect().center());
    check(!action->isChecked() && !hint->isVisible(), "button toggles off and hides operation hint");
    main.ui->textEdit_Show_CFmes->setPlainText("其他操作结果");
    main.ui->action_ed_cleangeo->trigger();
    check(main.ui->textEdit_Show_CFmes->toPlainText() == "其他操作结果", "distance cleanup preserves another operation's report");
    click(button, button->rect().center());
    if (cloud) {
        double p[3];
        cloud->GetMapper()->GetInput()->GetPoint(0, p);
        click(view, screen(view, renderer, p));
        click(view, screen(view, renderer, p));
    }
    main.ui->action_ed_cleangeo->trigger();
    check(main.ui->textEdit_Show_CFmes->toPlainText().isEmpty(), "geometry cleanup clears distance result");
    click(button, button->rect().center());
    main.ui->action_ed_cleangall->trigger();
    check(!action->isChecked(), "scene cleanup exits active measurement");
    click(button, button->rect().center());
    check(!action->isChecked(), "empty main scene cannot enter measurement");
    main.ui->action_ed_undo->trigger();
    click(button, button->rect().center());
    check(action->isChecked(), "can measure cloud restored by undo");
    auto actors = renderer->GetActors();
    actors->InitTraversal();
    while (auto actor = actors->GetNextActor()) {
        if (actor->GetPickable() && actor->GetMapper() && actor->GetMapper()->GetInput()
            && actor->GetMapper()->GetInput()->GetNumberOfPoints() > 100) {
            double p[3];
            actor->GetMapper()->GetInput()->GetPoint(0, p);
            click(view, screen(view, renderer, p));
            click(view, screen(view, renderer, p));
            break;
        }
    }
    main.ui->action_ed_redo->trigger();
    check(!action->isChecked() && main.ui->textEdit_Show_CFmes->toPlainText().contains("已重做"),
          "measurement does not consume undo history / redo clears its visuals");
    main.close();
    pump();
}

int main(int argc, char** argv)
{
    mathTests();
    if (argc > 1 && (std::string(argv[1]) == "--gui" || std::string(argv[1]) == "--startup")) {
        QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
        QApplication app(argc, argv);
        app.setQuitOnLastWindowClosed(false);
        startupCameraTests();
        if (std::string(argv[1]) == "--gui") {
            guiTests();
            mainWindowTests();
        }
    }
    std::cout << "[RESULT] pass=" << passed << " fail=" << failed << std::endl;
    return failed == 0 ? 0 : 1;
}
