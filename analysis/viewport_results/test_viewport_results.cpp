#include "CloudForgeAnalyzer.h"
#include "Basic/ViewportResultOverlay.h"
#include "Basic/WeldPreparationOverlay.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QThread>
#include <vtkCamera.h>
#include <vtkCallbackCommand.h>
#include <vtkDataSet.h>
#include <iostream>
#include <iomanip>
#include <cstdlib>

static int passed = 0, failed = 0;
static void check(bool ok, const char* name)
{
    ok ? ++passed : ++failed;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
}

static void settle(int ms = 100)
{
    QElapsedTimer timer;
    timer.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (timer.elapsed() < ms);
}

static void mouse(QWidget* view, QEvent::Type type, QPointF pos, Qt::MouseButton button, Qt::MouseButtons buttons)
{
    QMouseEvent event(type, pos, QPointF(view->mapToGlobal(pos.toPoint())), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    const QString suffix = qEnvironmentVariable("RESULT_CAPTURE_SUFFIX", "");
    auto capture = [&](QWidget* widget, const QString& name) {
        settle();
        check(widget->grab().save("analysis/viewport_results/ui_evidence/" + name + suffix + ".png"),
              name.toUtf8().constData());
    };
    {
        QWidget view;
        view.resize(600, 400);
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        ViewportResultOverlay results(&view);
        WeldPreparationOverlay labels(&view, renderer, &results);
        check(!renderer->IsActiveCameraCreated(), "constructing result displays preserves lazy camera creation");
        check(results.isHidden(), "empty result panel starts hidden");
        results.setSection("legend", {{}, {{"近件缝边", {}, QColor("#42f294")},
                                             {"远件缝边", {}, QColor("#ffda55")}}, 2});
        results.setSection("value", {"测量结果", {{"弧长", "123.456 mm", {}}}});
        results.setSection("value", {"测量结果", {{"弧长", "234.567 mm", {}}}});
        check(results.sections_.size() == 2 && results.sections_[1].section.rows[0].value == "234.567 mm",
              "same owner replaces results without accumulating cards");
        view.show();
        settle();
        QWidget hint(&view);
        hint.setGeometry(12, 12, 200, 45);
        results.avoidWidget(&hint);
        hint.show();
        settle();
        check(results.y() > hint.geometry().bottom(), "result panel avoids active tool hints");
        hint.hide();
        settle();
        check(results.y() == 12, "result panel returns after tool hint closes");
        view.resize(240, 400);
        settle();
        check(results.width() == 216 && view.rect().contains(results.geometry()), "narrow viewport reflows legend and result rows");
        capture(&view, "narrow_panel");
        results.removeSection("legend");
        check(results.sections_.size() == 1 && !results.isHidden(), "clearing one owner preserves other results");
        results.clear();
        check(results.occupiedRect().isEmpty() && results.isHidden(), "clearing all results releases occupied area");
        check(!renderer->IsActiveCameraCreated(), "result lifecycle never accesses camera");
    }

    auto cylinder = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int z = -5; z <= 5; ++z) for (int i = 0; i <= 120; ++i) {
        const double angle = -.6 + i * .01;
        cylinder->emplace_back(100 * std::cos(angle), 100 * std::sin(angle), z);
    }
    auto coeff = pcl::make_shared<pcl::ModelCoefficients>();
    coeff->values = {0, 0, 0, 0, 0, 1, 100};
    MeasureArc::ArcParams params;
    params.method = MeasureArc::BSPLINE_LSQ;
    MeasureArc arc(cylinder, coeff, nullptr, params);
    arc.setAutoBuildActor(false);
    arc.compute();
    check(arc.success && std::isfinite(arc.arcLength), "existing cylinder arc calculation succeeds on analytic patch");
    if (!arc.success) return 1;
    const double arcLength = arc.arcLength;
    arc.buildVisualizationActor();
    auto curve = arc.getVisualizationActor();
    auto curveData = curve->GetMapper()->GetInput();
    const auto curvePoints = curveData->GetNumberOfPoints();

    auto plane = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int x = -20; x <= 20; ++x) for (int y = -10; y <= 10; ++y) plane->emplace_back(x, y, 0);
    GeodesicArcMeasurer geodesic(plane, 3);
    auto path = geodesic.measureArcLength(pcl::PointXYZ(-20, 0, 0), pcl::PointXYZ(20, 0, 0));
    check(path.success && std::abs(path.arc_length - 40) < 1e-4, "existing geodesic algorithm measures analytic planar path");
    if (!path.success) return 1;
    const auto originalPath = path.path_points;
    const double geodesicLength = path.arc_length;
    std::cout << "[VALUES] cylinder=" << std::setprecision(17) << arcLength << " geodesic=" << geodesicLength << std::endl;

    CloudForgeAnalyzer main;
    main.resize(1450, 950);
    main.show();
    settle();
    main.ClearAllPointCloud();
    main.AddPointCloud("cylinder", cylinder, ColorManager(210, 220, 230));
    main.AddPointCloud("plane", plane, ColorManager(100, 150, 160));
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    renderer->ResetCamera();
    auto camera = renderer->GetActiveCamera();
    double position[3]; camera->GetPosition(position);
    auto interactorStyle = view->interactor()->GetInteractorStyle();
    auto results = main.m_resultOverlay.get();
    main.VisualizeArcMeasurement(curve, arcLength);
    main.VisualizeGeodesicMeasurement(geodesic, path);
    check(results->sections_.size() == 2 && results->sections_[0].section.title == "圆柱弧长"
          && results->sections_[0].section.rows[0].value == QString::number(arcLength, 'f', 3) + " mm"
          && results->sections_[1].section.title == "测地线弧长"
          && results->sections_[1].section.rows[0].value == "40.000 mm", "real arc results use common Chinese panel with mm units");
    check(arc.arcLength == arcLength && path.arc_length == geodesicLength && path.path_points == originalPath
          && curve->GetMapper()->GetInput() == curveData && curveData->GetNumberOfPoints() == curvePoints,
          "result display preserves computed values and curve geometry exactly");
    check(std::equal(position, position + 3, camera->GetPosition())
          && interactorStyle == view->interactor()->GetInteractorStyle(), "showing results preserves camera and interactor");
    check(results->testAttribute(Qt::WA_TransparentForMouseEvents) && !curve->GetPickable(),
          "panel and arc curve cannot intercept point picking");
    main.VisualizeArcMeasurement(curve, arcLength);
    const auto actorCount = main.m_geodesicVisualizationActors.size();
    main.VisualizeGeodesicMeasurement(geodesic, path);
    check(results->sections_.size() == 2 && main.ActorMap.count("arc_spline_result") == 1
          && main.m_geodesicVisualizationActors.size() == actorCount, "repeated measures replace matching panel and geometry");
    auto failedPath = path; failedPath.success = false;
    main.VisualizeGeodesicMeasurement(geodesic, failedPath);
    main.VisualizeArcMeasurement(nullptr, 1);
    check(results->sections_.size() == 2, "unsuccessful results do not create or overwrite cards");
    capture(&main, "arc_results");
    const QPoint start = results->geometry().center();
    check(view->childAt(start) != results, "mouse hit testing passes through result panel to viewport");
    mouse(view, QEvent::MouseButtonPress, start, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, start + QPoint(55, 25), Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, start + QPoint(55, 25), Qt::LeftButton, Qt::NoButton);
    settle();
    check(!std::equal(position, position + 3, camera->GetPosition()), "dragging over panel still rotates 3D scene");
    main.m_weldPrepOverlay->setAnnotations({{{100, 0, 0}, {}, "最大径向阶差 3.331 mm", "整体中值 2.999 mm", QColor("#68b7ff")}});
    check(results->sections_.size() == 3 && results->sections_.back().section.columns == 2,
          "weld legend joins common panel without covering arc results");
    capture(&main, "combined_results");
    main.m_weldPrepOverlay->clear();
    check(results->sections_.size() == 2, "weld cleanup preserves both arc results");
    main.ui->action_ed_cleangeodetic->trigger();
    check(results->sections_.size() == 1 && main.m_geodesicVisualizationActors.empty(),
          "geodesic cleanup removes its result and geometry together");
    main.VisualizeGeodesicMeasurement(geodesic, path);
    main.Slot_ed_clean2DActor_Triggered();
    check(results->isHidden() && !main.m_geodesicVisualizationActors.empty(), "clear 2D action clears fixed results");
    main.VisualizeArcMeasurement(curve, arcLength);
    main.VisualizeGeodesicMeasurement(geodesic, path);
    main.DelePointCloud("plane");
    check(results->isHidden() && main.m_geodesicVisualizationActors.empty()
          && !main.ActorMap.count("arc_spline_result"), "point cloud deletion invalidates arc geometry and displayed values");
    main.VisualizeArcMeasurement(curve, arcLength);
    main.clearAllActors();
    check(results->isHidden(), "actor cleanup also clears cylinder result card");
    main.VisualizeArcMeasurement(curve, arcLength);
    main.VisualizeGeodesicMeasurement(geodesic, path);
    main.ui->action_ed_cleangeo->trigger();
    check(results->isHidden() && main.m_geodesicVisualizationActors.empty()
          && !main.ActorMap.count("arc_spline_result"), "clear geometry action clears matching Qt and VTK results together");
    main.VisualizeArcMeasurement(curve, arcLength);
    main.VisualizeGeodesicMeasurement(geodesic, path);
    main.Slot_ed_undo_Triggered();
    check(results->isHidden() && main.m_geodesicVisualizationActors.empty(), "undo invalidates stale measurements in both display systems");
    main.VisualizeArcMeasurement(curve, arcLength);
    main.VisualizeGeodesicMeasurement(geodesic, path);
    main.ui->action_ed_cleangall->trigger();
    check(results->isHidden() && main.ActorMap.empty() && main.m_geodesicVisualizationActors.empty(),
          "clear all action leaves no Qt result or VTK measurement geometry");

    auto heights = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int x = -10; x <= 10; ++x) for (int y = -8; y <= 8; ++y)
        heights->emplace_back(x, y, 2 + .1f * x);
    MeasureHeight height(heights, plane);
    check(height.measure(), "height input produces valid measurement");
    main.visualizeMeasurementResults(height, heights, plane);
    settle();
    QMainWindow* heightWindow = nullptr;
    for (auto widget : QApplication::topLevelWidgets())
        if (widget->windowTitle() == "高度测量可视化") heightWindow = qobject_cast<QMainWindow*>(widget);
    check(heightWindow != nullptr, "height retains its independent Qt visualization window");
    if (heightWindow) {
        auto panel = heightWindow->findChild<ViewportResultOverlay*>("viewportResultOverlay");
        check(panel && panel->sections_.size() == 1 && panel->sections_[0].section.rows.size() == 3
              && panel->sections_[0].section.rows[0].value == QString::number(height.GetMaxDistance(), 'f', 3) + " mm"
              && panel->sections_[0].section.rows[2].value == QString::number(height.GetMeanDistance(), 'f', 3) + " mm",
              "height shows unchanged maximum minimum mean in common result style");
        capture(heightWindow, "height_results");
        QPointer<QWidget> guard(heightWindow);
        heightWindow->activateWindow();
        heightWindow->centralWidget()->setFocus();
        settle();
        QKeyEvent quit(QEvent::KeyPress, Qt::Key_Q, Qt::NoModifier, "q");
        QApplication::sendEvent(heightWindow->centralWidget(), &quit);
        QApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
        check(guard.isNull(), "height Q shortcut closes window and releases viewer and UI");
        if (guard) guard->close();
    }
    int renders = 0;
    auto observer = vtkSmartPointer<vtkCallbackCommand>::New();
    observer->SetClientData(&renders);
    observer->SetCallback([](vtkObject*, unsigned long, void* data, void*) { ++*static_cast<int*>(data); });
    const auto tag = renderer->AddObserver(vtkCommand::EndEvent, observer);
    settle(350);
    check(renders <= 1, "result component does not introduce an idle render loop");
    renderer->RemoveObserver(tag);
    main.close();
    std::cout << "pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
