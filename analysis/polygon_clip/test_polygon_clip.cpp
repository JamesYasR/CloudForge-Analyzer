#include "CloudForgeAnalyzer.h"
#include "Basic/PolygonClipTool.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QMouseEvent>
#include <QPushButton>
#include <QThread>
#include <QToolBar>
#include <vtkDataSet.h>
#include <vtkLODActor.h>
#include <vtkPropCollection.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

static int passed = 0, failed = 0;
static void check(bool condition, const char* name)
{
    condition ? ++passed : ++failed;
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << std::endl;
}

static void pump(int ms = 50)
{
    QElapsedTimer clock;
    clock.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (clock.elapsed() < ms);
}

template<class F> static bool waitFor(F condition, int ms = 10000)
{
    QElapsedTimer clock;
    clock.start();
    while (!condition() && clock.elapsed() < ms) pump(10);
    return condition();
}

static QString capture(const char* name)
{
    const char* suffix = std::getenv("POLYGON_CAPTURE_SUFFIX");
    return QString("analysis/polygon_clip/%1%2.png").arg(name, suffix ? suffix : "");
}

static void mouse(QWidget* view, QEvent::Type type, QPointF point, Qt::MouseButton button = Qt::NoButton,
                  Qt::MouseButtons buttons = Qt::NoButton)
{
    QMouseEvent event(type, point, QPointF(view->mapToGlobal(point.toPoint())), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}

static void click(QWidget* view, QPointF point, Qt::MouseButton button = Qt::LeftButton)
{
    mouse(view, QEvent::MouseButtonPress, point, button, button);
    mouse(view, QEvent::MouseButtonRelease, point, button);
    pump(10);
}

static void key(QWidget* view, int key, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
{
    QKeyEvent event(QEvent::KeyPress, key, modifiers);
    QApplication::sendEvent(view, &event);
    pump(10);
}

static QWidget* controls(QWidget* view)
{
    auto dock = view->window()->findChild<QDockWidget*>("polygonClipDock");
    return dock ? dock->widget() : view;
}
static QPushButton* button(QWidget* view, const char* name) { return controls(view)->findChild<QPushButton*>(name); }
static void quad(QWidget* view)
{
    for (const auto& point : {QPointF(.50, .20), QPointF(.80, .20), QPointF(.80, .80), QPointF(.50, .80)})
        click(view, {point.x() * view->width(), point.y() * view->height()});
    button(view, "polygonClose")->click();
    pump();
}

static CloudClipper::Projection identity()
{
    CloudClipper::Projection projection;
    for (int i = 0; i < 4; ++i) projection.matrix[i * 4 + i] = 1;
    return projection;
}

static bool legacyContains(const std::vector<CloudClipper::Point2D>& polygon, double x, double y)
{
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ((polygon[i][1] > y) != (polygon[j][1] > y)
            && x < (polygon[j][0] - polygon[i][0]) * (y - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]) inside = !inside;
    }
    return inside;
}

static void algorithmTests()
{
    using C = CloudClipper;
    std::vector<C::Point2D> square{{.25,.25},{.75,.25},{.75,.75},{.25,.75}};
    check(C::validatePolygon(square).empty(), "valid square");
    check(C::contains(square, {.5,.5}) && !C::contains(square, {.1,.5}), "known inside and outside");
    check(C::contains(square, {.25,.25}) && C::contains(square, {.25,.5}), "boundary points retained inside");
    auto reversed = square;
    std::reverse(reversed.begin(), reversed.end());
    check(C::validatePolygon(reversed).empty() && C::contains(reversed, {.5,.5}), "clockwise and counterclockwise contours");
    std::vector<C::Point2D> concave{{.1,.1},{.9,.1},{.9,.4},{.4,.4},{.4,.9},{.1,.9}};
    check(C::validatePolygon(concave).empty() && C::contains(concave, {.2,.8}) && !C::contains(concave, {.8,.8}), "concave polygon");
    check(!C::validatePolygon({{0,0},{1,1},{0,1},{1,0}}).empty(), "reject crossing contour");
    check(!C::validatePolygon({{0,0},{.5,0},{1,0}}).empty(), "reject zero-area contour");
    check(!C::validatePolygon({{0,0},{0,0},{1,1}}).empty(), "reject adjacent repeated points");
    check(!C::validatePolygon({{0,0},{1,1}}).empty(), "reject fewer than three vertices");
    check(!C::validatePolygon({{NAN,0},{1,0},{0,1}}).empty(), "reject nonfinite vertices");
    auto cloud = pcl::make_shared<C::Cloud>();
    for (int y = -4; y <= 4; ++y) for (int x = -4; x <= 4; ++x) cloud->push_back({x*.225f,y*.225f,.5f});
    cloud->header.frame_id = "fixture";
    cloud->sensor_origin_ = Eigen::Vector4f(1,2,3,0);
    C::Request request{cloud, square, identity()};
    auto result = C::compute(request);
    check(result.success && result.inside->size() == 25 && result.outside->size() == 56, "known 9x9 grid partition 25 / 56");
    check(result.inside->header.frame_id == "fixture" && result.outside->sensor_origin_ == cloud->sensor_origin_, "preserve cloud metadata");
    check(cloud->size() == 81 && result.inside->width == 25 && result.inside->height == 1, "input immutable / output dimensions valid");
    auto cancelled = C::compute(request, [](int,int,const std::string&) { return false; });
    check(cancelled.cancelled && !cancelled.success && cancelled.inside->empty(), "cancel does not return an applicable result");
    auto empty = request;
    empty.cloud = pcl::make_shared<C::Cloud>();
    check(!C::compute(empty).success, "reject empty source cloud");
    auto badViewport = request;
    badViewport.projection.viewport = {0,0,0,1};
    check(!C::compute(badViewport).success, "reject zero-size viewport");
    auto transformed = pcl::make_shared<C::Cloud>();
    transformed->push_back({-.75f,0,.5f});
    auto translated = identity();
    translated.matrix[3] = .75;
    check(C::compute({transformed,square,translated}).inside->size() == 1, "world transform included in projection");
    auto depth = pcl::make_shared<C::Cloud>();
    depth->push_back({0,0,.5f}); depth->push_back({0,0,-.5f}); depth->push_back({0,0,2}); depth->push_back({NAN,0,.5f});
    auto depthResult = C::compute({depth,square,identity()});
    check(depthResult.inside->size() == 1 && depthResult.outside->size() == 3 && !depthResult.outside->is_dense, "clip depth and preserve nonfinite outside points");
    auto perspective = identity();
    perspective.matrix[14] = 1; perspective.matrix[15] = 0;
    check(C::compute({depth,square,perspective}).inside->size() == 2, "perspective / reject points behind the eye");
    auto random = pcl::make_shared<C::Cloud>();
    for (int i = 0; i < 1000; ++i) random->push_back({float(std::sin(i * .713)*.95),float(std::cos(i*.349)*.95),.5f});
    auto reference = C::compute({random,concave,identity()});
    size_t index = 0;
    bool same = reference.success;
    for (const auto& p : *random) if (legacyContains(concave,.5+.5*p.x,.5-.5*p.y)) {
        if (index >= reference.inside->size() || (*reference.inside)[index].getVector3fMap() != p.getVector3fMap()) same = false;
        ++index;
    }
    check(same && index == reference.inside->size(), "legacy ray-crossing baseline: identical coordinates and order away from boundaries");
    auto large = pcl::make_shared<C::Cloud>();
    large->reserve(2000000);
    for (int y = 0; y < 1000; ++y) for (int x = 0; x < 2000; ++x)
        large->push_back({(x-1000)/1000.f,(y-500)/500.f,.5f});
    QElapsedTimer timer;
    timer.start();
    auto largeResult = C::compute({large,square,identity()});
    std::cout << "[PERF] 2M points partition " << timer.elapsed() << " ms" << std::endl;
    check(largeResult.success && largeResult.inside->size() == 501501
          && largeResult.inside->size() + largeResult.outside->size() == large->size(), "2M points exact partition / no loss");
}

static size_t registeredPoints(vtkRenderer* renderer)
{
    size_t count = 0;
    auto props = renderer->GetViewProps();
    props->InitTraversal();
    while (auto prop = props->GetNextProp()) {
        auto actor = vtkLODActor::SafeDownCast(prop);
        if (actor && actor->GetMapper() && actor->GetMapper()->GetInput()) count += actor->GetMapper()->GetInput()->GetNumberOfPoints();
    }
    return count;
}

static void toolTests()
{
    QVTKOpenGLNativeWidget view;
    view.resize(1200,800);
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);
    auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>(renderer,window,"clip-test",false);
    view.setRenderWindow(window);
    viewer->setupInteractor(view.interactor(), window);
    const bool created = renderer->IsActiveCameraCreated();
    PolygonClipTool tool(&view,renderer,window);
    check(created == bool(renderer->IsActiveCameraCreated()), "tool construction does not create or reset startup camera");
    auto style = view.interactor()->GetInteractorStyle();
    auto cloud = pcl::make_shared<CloudClipper::Cloud>();
    for (int y = -10; y <= 10; ++y) for (int x = -10; x <= 10; ++x) cloud->push_back({float(x),float(y),0});
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "grid");
    viewer->setBackgroundColor(0, .3, .4);
    renderer->ResetCamera();
    view.show(); pump(200);
    auto actor = viewer->getCloudActorMap()->at("grid").actor;
    auto connection = QObject::connect(&tool,&PolygonClipTool::previewRequested,[&]() {
        auto ticket = tool.ticket();
        tool.finishPreview(std::make_shared<CloudClipper::Result>(CloudClipper::compute(tool.request())),ticket);
    });
    check(!tool.start({}), "empty target list rejected");
    check(tool.start({{"grid",cloud,actor}}), "start without a separate window");
    auto panel = controls(&view)->findChild<QFrame*>("polygonClipPanel");
    if (!panel) panel = qobject_cast<QFrame*>(controls(&view));
    check(panel && panel->isVisible(), "visible mouse-operated control panel");
    quad(&view);
    check(button(&view,"polygonPreview")->isEnabled(), "close contour with a button");
    button(&view,"polygonPreview")->click(); pump();
    check(tool.result() && tool.result()->inside->size() > 0 && tool.result()->outside->size() > 0, "in-memory preview splits actual visible cloud");
    check(actor->GetVisibility() == 0 && button(&view,"polygonApply")->isEnabled(), "preview highlights classification without replacing source");
    view.grab().save(capture("preview"));
    controls(&view)->findChild<QComboBox*>("polygonMode")->setCurrentIndex(1);
    check(tool.keepMode() == PolygonClipTool::KeepMode::Outside && tool.canApply(), "switch to retaining outside without recomputing");
    controls(&view)->findChild<QComboBox*>("polygonMode")->setCurrentIndex(2);
    check(tool.keepMode() == PolygonClipTool::KeepMode::Both && tool.canApply(), "split mode available");
    double position[3]; renderer->GetActiveCamera()->GetPosition(position);
    QPointF vertex(view.width()*.5, view.height()*.2);
    mouse(&view,QEvent::MouseButtonPress,vertex,Qt::LeftButton,Qt::LeftButton);
    mouse(&view,QEvent::MouseMove,vertex + QPointF(-35,-15),Qt::NoButton,Qt::LeftButton);
    mouse(&view,QEvent::MouseButtonRelease,vertex + QPointF(-35,-15),Qt::LeftButton);
    check(!tool.result() && actor->GetVisibility() == 1 && !button(&view,"polygonApply")->isEnabled(), "dragging a vertex invalidates preview and restores source");
    check(view.interactor()->GetInteractorStyle() == style && std::equal(position,position+3,renderer->GetActiveCamera()->GetPosition()), "drawing does not change interactor or camera");
    auto before = tool.request().polygon.size();
    const auto points = tool.request().polygon;
    click(&view,{(points[0][0]+points[1][0])*.5*view.width(),(points[0][1]+points[1][1])*.5*view.height()});
    check(tool.request().polygon.size() == before + 1, "click an edge to insert a vertex");
    button(&view,"polygonUndo")->click();
    check(tool.request().polygon.size() == before && button(&view,"polygonPreview")->isEnabled(), "undo insertion restores the previous closed contour");
    button(&view,"polygonReset")->click();
    check(tool.request().polygon.empty(), "clear contour with a button");
    quad(&view);
    button(&view,"polygonPreview")->click();
    view.resize(1250,820); pump();
    check(!tool.result() && actor->GetVisibility(), "resize invalidates cached preview");
    button(&view,"polygonPreview")->click();
    button(&view,"polygonNavigate")->click();
    QPointF p(view.width()*.8,view.height()*.5);
    click(&view,p);
    check(!tool.result(), "navigation restores original cloud before moving view");
    check(view.interactor()->GetInteractorStyle() == style, "navigation uses original VTK interactor style");
    button(&view,"polygonDraw")->click();
    key(&view,Qt::Key_S);
    check(!controls(&view)->findChild<QCheckBox*>("polygonContourVisible")->isChecked(), "S shortcut retained");
    key(&view,Qt::Key_R);
    check(tool.request().polygon.empty(), "R shortcut retained");
    for (const auto& p : {QPointF(.5,.2),QPointF(.8,.8),QPointF(.5,.8),QPointF(.8,.2)}) click(&view,{p.x()*view.width(),p.y()*view.height()});
    button(&view,"polygonClose")->click();
    check(!button(&view,"polygonPreview")->isEnabled() && controls(&view)->findChild<QLabel*>("polygonHint")->text().contains("交叉"), "self-intersection explained and preview disabled");
    button(&view,"polygonReset")->click();
    quad(&view);
    QObject::disconnect(connection);
    button(&view,"polygonPreview")->click();
    auto ticket = tool.ticket();
    auto request = tool.request();
    check(tool.isBusy(), "preview pending state");
    button(&view,"polygonExit")->click();
    tool.finishPreview(std::make_shared<CloudClipper::Result>(CloudClipper::compute(request)),ticket);
    check(!tool.isActive() && !tool.result() && actor->GetVisibility(), "late preview result ignored after cancellation");
    viewer->addPointCloud<pcl::PointXYZ>(cloud,"second");
    auto second = viewer->getCloudActorMap()->at("second").actor;
    tool.start({{"grid",cloud,actor},{"second",cloud,second}});
    quad(&view);
    button(&view,"polygonPreview")->click();
    tool.finishPreview(std::make_shared<CloudClipper::Result>(CloudClipper::compute(tool.request())),tool.ticket());
    controls(&view)->findChild<QComboBox*>("polygonSource")->setCurrentIndex(1);
    check(actor->GetVisibility() && second->GetVisibility() && !tool.result() && tool.request().polygon.empty(),
          "switching source restores previous actor and clears contour");
    tool.stop();
    actor->GetProperty()->SetOpacity(0);
    check(!tool.start({{"grid",cloud,actor}}), "fully transparent cloud is not a clipping target");
    actor->GetProperty()->SetOpacity(1);
    tool.start({{"grid",cloud,actor}});
    controls(&view)->findChild<QComboBox*>("polygonMode")->setCurrentIndex(0);
    for (const auto& p : {QPointF(.86,.86),QPointF(.96,.86),QPointF(.96,.96),QPointF(.86,.96)})
        click(&view,{p.x()*view.width(),p.y()*view.height()});
    button(&view,"polygonClose")->click();
    button(&view,"polygonPreview")->click();
    tool.finishPreview(std::make_shared<CloudClipper::Result>(CloudClipper::compute(tool.request())),tool.ticket());
    check(tool.result() && tool.result()->inside->empty() && !button(&view,"polygonApply")->isEnabled(),
          "empty retained region cannot be applied");
    controls(&view)->findChild<QComboBox*>("polygonMode")->setCurrentIndex(1);
    check(tool.canApply() && tool.result()->outside->size() == cloud->size(), "outside retention remains available when inside is empty");
    tool.stop();
    for (int i = 0; i < 15; ++i) { tool.start({{"grid",cloud,actor}}); tool.stop(); }
    check(view.interactor()->GetInteractorStyle() == style && actor->GetVisibility(), "repeated entry and cancel preserve original state");
    tool.start({{"grid",cloud,actor}});
    key(&view,Qt::Key_Escape);
    check(!tool.isActive(), "Esc exits without closing application");
    view.close();
}

static void mainTests()
{
    CloudForgeAnalyzer main;
    main.resize(1450,950);
    main.show(); pump(250);
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto style = view->interactor()->GetInteractorStyle();
    const size_t original = registeredPoints(renderer);
    auto entry = main.ui->mainToolBar->widgetForAction(main.ui->action_Clip);
    click(entry,entry->rect().center());
    auto dock = main.findChild<QDockWidget*>("polygonClipDock");
    check(main.ui->action_Clip->isChecked() && dock->isVisible(), "actual sidebar opens Qt clipping dock");
    check(main.dockWidgetArea(dock) == Qt::LeftDockWidgetArea && !dock->isFloating()
          && !view->isAncestorOf(dock), "clipping controls dock outside the VTK viewport");
    dock->setFloating(true); pump();
    check(dock->isFloating() && dock->height() < main.height() && main.ui->action_Clip->isChecked(), "compact floating Qt dock preserves clipping session");
    dock->grab().save(capture("floating_controls"));
    dock->setFloating(false); pump();
    check(!dock->isFloating() && main.ui->action_Clip->isChecked(), "Qt dock can return to the main window");
    key(controls(view),Qt::Key_Escape);
    check(!dock->isVisible() && !main.ui->action_Clip->isChecked(), "Esc from dock controls closes clipping session");
    click(entry,entry->rect().center());
    dock->close(); pump();
    check(!main.ui->action_Clip->isChecked() && !dock->isVisible(), "native dock close exits clipping session");
    click(entry,entry->rect().center());
    pump();
    quad(view);
    key(view,Qt::Key_Z,Qt::ControlModifier);
    check(main.ui->action_Clip->isChecked() && button(view,"polygonClose")->isEnabled() && registeredPoints(renderer) == original,
          "Ctrl+Z edits contour without triggering cloud undo");
    button(view,"polygonClose")->click();
    button(view,"polygonPreview")->click();
    check(waitFor([&]() { return button(view,"polygonApply")->isEnabled(); }), "background preview completes in main window");
    check(registeredPoints(renderer) == original, "preview does not modify cloud registry");
    main.grab().save(capture("main_preview"));
    double camera[3]; renderer->GetActiveCamera()->GetPosition(camera);
    button(view,"polygonApply")->click(); pump();
    const size_t inside = registeredPoints(renderer);
    check(inside > 0 && inside < original && !main.ui->action_Clip->isChecked(), "apply retains inside without crash");
    check(view->interactor()->GetInteractorStyle() == style && std::equal(camera,camera+3,renderer->GetActiveCamera()->GetPosition()), "application preserves original camera and interactor");
    main.grab().save(capture("main_applied"));
    main.ui->action_ed_undo->trigger(); pump();
    check(registeredPoints(renderer) == original, "single undo restores full original cloud");
    main.ui->action_ed_redo->trigger(); pump();
    check(registeredPoints(renderer) == inside, "single redo restores applied cut");
    main.ui->action_ed_undo->trigger(); pump();
    bool cycles = true;
    for (int i = 0; i < 6; ++i) {
        QElapsedTimer elapsed;
        elapsed.start();
        click(entry,entry->rect().center());
        controls(view)->findChild<QComboBox*>("polygonMode")->setCurrentIndex(i%3);
        quad(view);
        button(view,"polygonPreview")->click();
        if (!waitFor([&]() { return button(view,"polygonApply")->isEnabled(); })) {
            std::cout << "[CYCLE] " << i << " preview unavailable: "
                      << controls(view)->findChild<QLabel*>("polygonHint")->text().toStdString()
                      << " / " << controls(view)->findChild<QLabel*>("polygonCount")->text().toStdString()
                      << " elapsed=" << elapsed.elapsed() << std::endl;
            cycles = false;
            break;
        }
        button(view,"polygonApply")->click(); pump();
        const auto applied = registeredPoints(renderer);
        if (i%3 == 2 && applied != original) cycles = false;
        main.ui->action_ed_undo->trigger(); pump();
        if (registeredPoints(renderer) != original || view->interactor()->GetInteractorStyle() != style) cycles = false;
        std::cout << "[CYCLE] " << i << " applied=" << applied << " restored=" << registeredPoints(renderer)
                  << " expected=" << original << " elapsed=" << elapsed.elapsed() << std::endl;
    }
    check(cycles, "six preview/apply/undo cycles across all retention modes without crash or data loss");
    click(entry,entry->rect().center());
    quad(view);
    button(view,"polygonPreview")->click();
    button(view,"polygonExit")->click();
    waitFor([&]() { return !main.ui->action_cancel_task->isEnabled(); });
    check(!main.ui->action_Clip->isChecked() && registeredPoints(renderer) == original, "cancel during async preview preserves source");
    click(entry,entry->rect().center());
    main.ui->action_ed_cleangeo->trigger();
    check(!main.ui->action_Clip->isChecked() && !dock->isVisible(), "cleanup exits clipping without stale dock");
    main.close(); pump();
}

int main(int argc, char** argv)
{
    algorithmTests();
    if (argc > 1 && std::string(argv[1]) == "--gui") {
        QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
        QApplication app(argc,argv);
        app.setQuitOnLastWindowClosed(false);
        toolTests();
        mainTests();
    }
    std::cout << "[RESULT] pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
