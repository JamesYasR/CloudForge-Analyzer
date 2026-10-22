#include "CloudForgeAnalyzer.h"
#include "Basic/ViewportPointPicker.h"
#include "Basic/ViewportResultOverlay.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QThread>
#include <QCheckBox>
#include <QPushButton>
#include <vtkCamera.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkSphereSource.h>
#include <vtkPointPicker.h>
#include <vtkPropCollection.h>
#include <iostream>

static int passed = 0, failed = 0;
static void check(bool ok, const char* name)
{
    ok ? ++passed : ++failed;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
}
static void pump(int ms = 90)
{
    QElapsedTimer timer; timer.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (timer.elapsed() < ms);
}
static void mouse(QWidget* view, QEvent::Type type, QPoint pos, Qt::MouseButton button, Qt::MouseButtons buttons = {})
{
    QMouseEvent event(type, QPointF(pos), QPointF(view->mapToGlobal(pos)), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}
static void click(QWidget* view, QPoint pos, bool wait = true)
{
    mouse(view, QEvent::MouseButtonPress, pos, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, pos, Qt::LeftButton);
    if (wait) pump();
}
static QPoint screen(QWidget* view, vtkRenderer* renderer, const double* point)
{
    renderer->SetWorldPoint(point[0], point[1], point[2], 1);
    renderer->WorldToDisplay();
    auto p = renderer->GetDisplayPoint();
    const auto size = renderer->GetRenderWindow()->GetSize();
    return {qRound(p[0] * view->width() / size[0]), qRound((size[1] - 1 - p[1]) * view->height() / size[1])};
}

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    app.setQuitOnLastWindowClosed(false);
    CloudForgeAnalyzer main;
    main.resize(1450, 950);
    main.show(); pump();
    auto plane = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int x = -20; x <= 20; ++x) for (int y = -10; y <= 10; ++y) plane->emplace_back(x, y, 0);
    main.ClearAllPointCloud();
    main.AddPointCloud("plane", plane, ColorManager(220, 230, 235));
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto camera = renderer->GetActiveCamera();
    auto resetCamera = [&]() {
        camera->SetPosition(0, 0, 100); camera->SetFocalPoint(0, 0, 0); camera->SetViewUp(0, 1, 0);
        camera->ParallelProjectionOn(); camera->SetParallelScale(26);
        renderer->ResetCameraClippingRange(); view->renderWindow()->Render(); pump();
    };
    resetCamera();
    const double a[3] = {-15, 0, 0}, b[3] = {15, 0, 0};
    auto picker = main.m_pointPicker.get();
    auto style = view->interactor()->GetInteractorStyle();
    auto originalPicker = view->interactor()->GetPicker();
    const auto cursor = view->cursor();
    const bool tracking = view->hasMouseTracking();
    int callbacks = 0;
    auto callback = [&](const CloudForgeAnalyzer::PickedPoints&) { ++callbacks; };
    main.PickCloudPoints("plane", 2, "测地线弧长", callback);
    check(picker->isActive() && picker->isSelecting() && !main.ui->measure_line->isChecked(),
          "finite point selection shares distance picker without activating distance action");
    check(picker->picker_->GetPickList()->GetNumberOfItems() == 1, "selection restricts picker to chosen cloud actor");
    mouse(view, QEvent::MouseMove, screen(view, renderer, a), Qt::NoButton);
    pump();
    check(picker->points_[0]->GetVisibility() && !picker->lineActor_->GetVisibility(),
          "hover highlights source point without displaying a false straight geodesic");
    click(view, {2, view->height() - 2});
    check(picker->selectedPoints().empty(), "blank clicks do not become selected endpoints");
    const auto pa = screen(view, renderer, a), pb = screen(view, renderer, b);
    click(view, pa);
    double cameraBefore[3]; camera->GetPosition(cameraBefore);
    mouse(view, QEvent::MouseButtonPress, pb, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, pb + QPoint(55, 30), Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, pb, Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, pb, Qt::LeftButton);
    pump();
    check(picker->selectedPoints().size() == 1 && callbacks == 0, "dragging back to starting pixel does not select endpoint");
    check(!std::equal(cameraBefore, cameraBefore + 3, camera->GetPosition()), "original camera rotation stays available during finite selection");
    resetCamera();
    click(view, screen(view, renderer, b));
    check(callbacks == 1 && !picker->isActive(), "second click completes selection once without nested event loop");
    check(view->cursor().shape() == cursor.shape() && view->hasMouseTracking() == tracking
          && view->interactor()->GetInteractorStyle() == style && view->interactor()->GetPicker() == originalPicker,
          "selection restores cursor and mouse tracking while preserving interactor and its picker");
    check(!picker->points_[0]->GetVisibility() && !picker->labels_[0]->GetVisibility(), "temporary selection markers are removed after completion");

    main.PickCloudPoints("plane", 2, "测地线弧长", callback);
    click(view, screen(view, renderer, a));
    auto cancel = view->findChild<QPushButton*>("pointPickCancel");
    check(cancel && cancel->isVisible(), "cancel button is available without hotkeys");
    if (cancel) click(cancel, cancel->rect().center());
    check(!picker->isActive() && callbacks == 1 && picker->selectedPoints().empty(), "cancel button discards incomplete selection");
    main.PickCloudPoints("plane", 1, "曲率分割", callback);
    main.ui->textEdit_Show_CFmes->setFocus();
    QKeyEvent esc(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QApplication::sendEvent(main.ui->textEdit_Show_CFmes, &esc);
    check(!picker->isActive() && callbacks == 1, "Esc cancels selection from side panel focus");

    auto ghost = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ghost->emplace_back(a[0], a[1], 5);
    main.viewer->addPointCloud<pcl::PointXYZ>(ghost, "foreign_ghost");
    view->renderWindow()->Render();
    bool sourcePoint = false;
    main.PickCloudPoints("plane", 1, "圆柱弧长", [&](const auto& points) { sourcePoint = points.size() == 1 && points[0][2] == 0; });
    click(view, screen(view, renderer, a));
    check(sourcePoint, "single-point mode selects chosen source cloud rather than overlapping foreign actor");
    main.viewer->removePointCloud("foreign_ghost");

    main.PickCloudPoints("plane", 2, "测地线弧长", callback);
    click(view, screen(view, renderer, a), false);
    click(view, screen(view, renderer, b), false);
    main.ui->measure_line->trigger();
    pump();
    check(callbacks == 1 && picker->isActive() && !picker->isSelecting(), "switching tools cancels queued continuation of completed selection");
    picker->clear();

    QTimer dialogs;
    dialogs.setInterval(20);
    QObject::connect(&dialogs, &QTimer::timeout, [&]() {
        auto modal = QApplication::activeModalWidget();
        if (!modal) return;
        if (auto cloudDialog = qobject_cast<ChoseCloudDialog*>(modal)) {
            cloudDialog->selectAllCheckBoxes();
            cloudDialog->accept();
        }
        else if (auto option = qobject_cast<OptionBox*>(modal)) {
            const auto buttons = option->findChildren<QPushButton*>();
            for (auto button : buttons) if (button->text().contains("B样条") || button->text().contains("手动选择")) {
                button->click(); break;
            }
        }
        else if (auto cy = qobject_cast<ChoseCyDialog*>(modal)) {
            cy->selectAllCheckBoxes(); cy->accept();
        }
        else if (auto params = qobject_cast<ParamDialogBase*>(modal)) {
            if (qobject_cast<ParamDialogMeaGeodetic*>(params)) params->findChild<QLineEdit*>()->setText("3");
            for (auto button : params->findChildren<QPushButton*>()) if (button->text() == "确定") { button->click(); break; }
        }
    });
    dialogs.start();
    QTimer timeout;
    timeout.setSingleShot(true);
    QObject::connect(&timeout, &QTimer::timeout, [&]() { if (auto modal = qobject_cast<QDialog*>(QApplication::activeModalWidget())) modal->reject(); });
    timeout.start(10000);
    main.Tool_MeasureGeodisic();
    dialogs.stop(); timeout.stop();
    check(picker->isActive() && picker->isSelecting() && picker->targetPoints_ == 2, "real geodesic dialog flow returns to nonblocking viewport selection");
    resetCamera();
    click(view, screen(view, renderer, a));
    mouse(view, QEvent::MouseMove, screen(view, renderer, b), Qt::NoButton); pump();
    const QString suffix = qEnvironmentVariable("PICK_CAPTURE_SUFFIX", "");
    check(main.grab().save("analysis/point_picking/geodesic_selection" + suffix + ".png"), "geodesic selection screenshot");
    click(view, screen(view, renderer, b));
    QElapsedTimer computation; computation.start();
    while (main.IsTaskRunning() && computation.elapsed() < 20000) pump(20);
    pump();
    const auto points = picker->selectedPoints();
    check(points.size() == 2, "geodesic receives exactly two source coordinates");
    if (points.size() == 2) {
        GeodesicArcMeasurer baseline(plane, 3);
        const auto expected = baseline.measureArcLength(pcl::PointXYZ(points[0][0], points[0][1], points[0][2]),
                                                      pcl::PointXYZ(points[1][0], points[1][1], points[1][2]));
        const auto rows = main.m_resultOverlay->sections_;
        check(expected.success && std::abs(expected.arc_length - 30) < 1e-4 && !main.IsTaskRunning()
              && rows.size() == 1 && rows[0].section.rows[0].value == QString::number(expected.arc_length, 'f', 3) + " mm",
              "real picked endpoints produce unchanged geodesic result through async computation and common result panel");
    }
    check(main.grab().save("analysis/point_picking/geodesic_result" + suffix + ".png"), "geodesic result screenshot");

    dialogs.start(); timeout.start(10000);
    main.Slot_ph_CurvSeg_Triggered();
    dialogs.stop(); timeout.stop();
    check(picker->isActive() && picker->targetPoints_ == 1, "curvature segmentation entry uses same single-point selection");
    cancel->click();
    check(!main.IsTaskRunning(), "curvature point cancellation does not launch computation");
    auto coeff = pcl::make_shared<pcl::ModelCoefficients>(); coeff->values = {0, 0, 0, 0, 0, 1, 100};
    main.addCylinderResult("cylinder", coeff);
    dialogs.start(); timeout.start(10000);
    main.Tool_MeasureArc();
    dialogs.stop(); timeout.stop();
    check(picker->isActive() && picker->targetPoints_ == 1, "cylinder arc manual entry uses same single-point selection");
    cancel->click();
    check(!main.IsTaskRunning(), "cylinder manual point cancellation does not launch computation");

    for (int i = 0; i < 4; ++i) {
        main.PickCloudPoints("plane", 1, "选点", callback);
        click(view, screen(view, renderer, a));
    }
    check(callbacks == 5, "repeat sessions do not accumulate completion callbacks");
    main.PickCloudPoints("plane", 2, "测地线弧长", callback);
    main.DelePointCloud("plane"); pump();
    check(!picker->isActive() && callbacks == 5, "source removal cancels pending point selection");
    main.Slot_ed_undo_Triggered();
    main.PickCloudPoints("plane", 2, "测地线弧长", callback);
    main.close(); pump();
    check(!picker->isActive(), "closing main window ends active point selection");
    std::cout << "pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
