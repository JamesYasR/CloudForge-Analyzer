#include "cases.h"
#include "CloudForgeAnalyzer.h"
#include "Basic/HeightMeasureTool.h"
#include "Basic/ViewportResultOverlay.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QThread>
#include <vtkCamera.h>
#include <vtkCallbackCommand.h>
#include <vtkDataSet.h>
#include <vtkPolyDataMapper.h>
#include <fstream>
#include <iostream>

static int passed = 0, failed = 0;
static void check(bool ok, const char* name)
{
    ok ? ++passed : ++failed;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
}
static void pump(int ms = 100)
{
    QElapsedTimer timer; timer.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (timer.elapsed() < ms);
}
static void mouse(QWidget* view, QEvent::Type type, QPoint pos, Qt::MouseButton button, Qt::MouseButtons buttons = {})
{
    QMouseEvent event(type, QPointF(pos), QPointF(view->mapToGlobal(pos)), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}
static void click(QWidget* widget)
{
    const auto p = qobject_cast<QCheckBox*>(widget) ? QPoint(8, widget->height()/2) : widget->rect().center();
    mouse(widget, QEvent::MouseButtonPress, p, Qt::LeftButton, Qt::LeftButton);
    mouse(widget, QEvent::MouseButtonRelease, p, Qt::LeftButton);
    pump();
}
static void waitTask(CloudForgeAnalyzer& main)
{
    QElapsedTimer timer; timer.start();
    while (main.IsTaskRunning() && timer.elapsed() < 20000) pump(20);
    pump();
    check(!main.IsTaskRunning(), "height computation returns without blocking GUI");
}
static void pureTests()
{
    std::ifstream file("analysis/height_measurement/baseline_6969631.txt");
    std::string expected;
    bool same = true;
    for (int i = 0; i < 5; ++i) {
        auto item = heightCase(i); MeasureHeight measurement(item.measured, item.reference);
        std::vector<double> averages;
        const bool ok = measurement.measureIterative(averages, 10, 5);
        std::getline(file, expected);
        same &= ok && heightSnapshot(measurement, averages, measurement.coloredCloud()) == expected + '\n';
    }
    check(same, "all five old snapshots match bit-for-bit: extrema mean plane iterations RGB hash");
    auto item = heightCase(1);
    MeasureHeight cancel(item.measured, item.reference);
    cancel.setProgressCallback([](int current, int, const std::string&) { return current < 50; });
    std::vector<double> values;
    check(!cancel.measureIterative(values, 10, 5) && values.empty(), "calculation cooperatively cancels before iterative scan");
    cancel.setProgressCallback({});
    check(!cancel.measureIterative(values, 0, 5) && !cancel.measureIterative(values, 10, 0), "invalid iteration requests rejected");
    auto empty = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    MeasureHeight missing(empty, item.reference);
    check(!missing.measureIterative(values, 10, 5), "empty height cloud rejected");
}

int main(int argc, char** argv)
{
    pureTests();
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    app.setQuitOnLastWindowClosed(false);
    {
        QMainWindow window;
        auto view = new QWidget(&window); window.setCentralWidget(view);
        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        ViewportResultOverlay result(view);
        HeightMeasureTool tool(view, renderer, &result);
        check(!renderer->IsActiveCameraCreated(), "constructing inactive height tool preserves lazy camera");
    }
    CloudForgeAnalyzer main;
    main.resize(1450, 1000); main.show(); pump();
    auto item = heightCase(1);
    main.ClearAllPointCloud();
    main.AddPointCloud("a_measured", item.measured, ColorManager(220, 230, 235));
    main.AddPointCloud("b_reference", item.reference, ColorManager(90, 150, 210));
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto camera = renderer->GetActiveCamera();
    camera->SetPosition(34, -45, 30); camera->SetFocalPoint(0, 0, 1); camera->SetViewUp(0, 0, 1);
    renderer->ResetCameraClippingRange(); view->renderWindow()->Render();
    double position[3]; camera->GetPosition(position);
    auto style = view->interactor()->GetInteractorStyle();
    auto originalPicker = view->interactor()->GetPicker();
    auto tool = main.m_heightTool.get();
    auto dock = main.findChild<QDockWidget*>("heightMeasureDock");
    auto original = main.viewer->getCloudActorMap()->at("a_measured").actor;
    auto reference = main.viewer->getCloudActorMap()->at("b_reference").actor;
    const int visibility = original->GetVisibility();
    const auto ptr = main.CloudMap.at("a_measured");
    const auto sourceColor = main.ColorMap.at("a_measured");
    const QString suffix = qEnvironmentVariable("HEIGHT_CAPTURE_SUFFIX", "");
    auto capture = [&](const QString& name) { pump(); check(main.grab().save("analysis/height_measurement/" + name + suffix + ".png"), name.toUtf8().constData()); };
    auto run = [&]() {
        click(dock->findChild<QPushButton*>("heightMeasure")); waitTask(main);
    };
    auto open = [&]() { main.ui->measure_height->trigger(); pump(); };
    open();
    check(tool->isActive() && dock->isVisible() && !dock->isFloating()
          && main.dockWidgetArea(dock) == Qt::LeftDockWidgetArea, "height action opens native dock in main window without input dialogs");
    check(dock->findChild<QSpinBox*>("heightNeighbors")->value() == 10
          && dock->findChild<QSpinBox*>("heightIterations")->value() == 5, "original parameter defaults preserved");
    check(dock->findChild<QComboBox*>("heightMeasured")->currentText() == "a_measured"
          && dock->findChild<QComboBox*>("heightReference")->currentText() == "b_reference", "input roles visible in panel");
    check(std::equal(position, position+3, camera->GetPosition()), "opening dock preserves camera pose");
    run();
    check(tool->actors_.size() == 6 && !original->GetVisibility() && !reference->GetVisibility(), "height display uses temporary geometry and hides original actors");
    check(main.m_resultOverlay->sections_.size() == 1
          && main.m_resultOverlay->sections_[0].section.rows[0].value == "2.750 mm"
          && main.m_resultOverlay->sections_[0].section.rows[1].value == "1.250 mm"
          && tool->table_->rowCount() == 5, "extrema and all iterative averages shown in panel and shared result overlay");
    check(main.ui->textEdit_Show_CFmes->toPlainText().contains("平均高度: 2.000 mm")
          && main.ui->textEdit_Show_CFmes->toPlainText().contains("第 5 次平均高度"), "height report retains complete measurement results with units");
    check(main.ui->textEdit_Show_CFmes->toPlainText().contains("绿色低、红色高")
          && main.ui->textEdit_Show_CFmes->toPlainText().contains("黄色线表示最大高度"), "color explanation appears in right result report");
    check(!dock->findChild<QLabel*>("heightHint"), "measurement panel has no status hint row");
    check(main.CloudMap.at("a_measured") == ptr && main.ColorMap.at("a_measured").r == sourceColor.r
          && (*ptr)[0].z == (*item.measured)[0].z, "source pointers coordinates and colors preserved");
    check(std::equal(position, position+3, camera->GetPosition()) && view->interactor()->GetInteractorStyle() == style
          && view->interactor()->GetPicker() == originalPicker, "height calculation and display preserve camera and VTK interaction");
    bool pickable = false;
    for (auto actor : tool->actors_) pickable |= actor->GetPickable();
    check(!pickable, "height plane lines and markers cannot intercept cloud picking");
    int independent = 0;
    for (auto widget : QApplication::topLevelWidgets()) if (widget->windowTitle() == "高度测量可视化") ++independent;
    check(independent == 0, "measurement does not create independent VTK window");
    auto data = tool->heatmapActor_->GetMapper()->GetInput();
    check(data->GetNumberOfPoints() == static_cast<vtkIdType>(item.measured->size()), "heatmap contains every original measured point");
    auto planeData = tool->planeActor_->GetMapper()->GetInput();
    bool onPlane = true;
    for (vtkIdType i=0;i<planeData->GetNumberOfPoints();++i) { double p[3]; planeData->GetPoint(i,p); onPlane &= std::abs(p[2]) < 1e-6; }
    check(onPlane, "finite reference plane lies on fitted surface");
    int renders = 0;
    auto observer = vtkSmartPointer<vtkCallbackCommand>::New(); observer->SetClientData(&renders);
    observer->SetCallback([](vtkObject*, unsigned long, void* data, void*) { ++*static_cast<int*>(data); });
    const auto tag = renderer->AddObserver(vtkCommand::EndEvent, observer);
    pump(350);
    check(renders <= 1, "height display introduces no idle render loop");
    renderer->RemoveObserver(tag);
    auto maxLine = tool->actors_[4]->GetMapper()->GetInput();
    double p[3], q[3]; maxLine->GetPoint(0,p); maxLine->GetPoint(1,q);
    check(std::abs(std::hypot(p[0]-q[0], p[1]-q[1], p[2]-q[2])-2.75) < 1e-6, "maximum height dimension joins true source and orthogonal projection");
    capture("main_result");
    click(dock->findChild<QCheckBox*>("heightColors"));
    check(original->GetVisibility() && !tool->heatmapActor_->GetVisibility(), "disabling height colors displays original source appearance");
    click(dock->findChild<QCheckBox*>("heightColors"));
    click(dock->findChild<QCheckBox*>("heightPlane"));
    check(!tool->planeActor_->GetVisibility() && !tool->referenceActor_->GetVisibility(), "reference visibility switch applies to surface and reference points");
    click(dock->findChild<QCheckBox*>("heightPlane"));
    click(dock->findChild<QCheckBox*>("heightLines"));
    check(!tool->linesActor_->GetVisibility(), "auxiliary height lines can be hidden independently");
    click(dock->findChild<QCheckBox*>("heightLines"));
    const auto start = view->rect().center();
    mouse(view, QEvent::MouseButtonPress, start, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, start+QPoint(55,25), Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, start+QPoint(55,25), Qt::LeftButton); pump();
    check(!std::equal(position,position+3,camera->GetPosition()), "model rotation available while result dock is open");
    run();
    check(tool->actors_.size() == 6, "remeasure replaces temporary actors rather than accumulating geometry");
    dock->setFloating(true); pump();
    check(dock->isFloating(), "height controls can float without new VTK view");
    check(dock->grab().save("analysis/height_measurement/floating_controls"+suffix+".png"), "floating controls capture");
    QKeyEvent esc(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QApplication::sendEvent(dock->findChild<QPushButton*>("heightMeasure"), &esc); pump();
    check(!tool->isActive() && tool->actors_.empty() && original->GetVisibility() == visibility
          && !main.ui->measure_height->isChecked() && main.m_resultOverlay->isHidden(), "Esc from floating controls restores sources and clears Qt and VTK overlays");
    dock->setFloating(false);
    open(); run();
    click(dock->findChild<QPushButton*>("heightClear"));
    check(tool->isActive() && tool->actors_.empty() && tool->table_->rowCount() == 0 && original->GetVisibility(), "clear result restores source while keeping panel ready");
    run(); main.ui->action_ed_clean2DActor->trigger();
    check(tool->isActive() && !tool->actors_.empty() && main.m_resultOverlay->isHidden(), "2D cleanup removes fixed annotation and preserves height geometry");
    main.ui->action_ed_cleangeo->trigger();
    check(!tool->isActive() && tool->actors_.empty() && original->GetVisibility(), "geometry cleanup removes height view and restores original actor");
    open(); run(); dock->close(); pump();
    check(!tool->isActive() && tool->actors_.empty(), "native dock close ends measurement session");
    open();
    QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::ControlModifier);
    QApplication::sendEvent(view, &enter); waitTask(main);
    check(!tool->actors_.empty(), "Ctrl Enter starts height measurement without modal dialogs");
    QKeyEvent quit(QEvent::KeyPress, Qt::Key_Q, Qt::NoModifier);
    QApplication::sendEvent(view, &quit); pump();
    check(!tool->isActive() && tool->actors_.empty(), "original Q exit hotkey retained in main viewport");

    open();
    const auto pending = tool->beginMeasurement();
    tool->stop();
    auto late = std::make_shared<HeightMeasureTool::Result>(); late->success = true;
    check(!tool->finish(late, pending.ticket, false) && tool->actors_.empty(), "late completion cannot recreate closed measurement overlays");
    open();
    main.StartHeightMeasurement();
    main.CancelCurrentTask(); waitTask(main);
    check(tool->isActive() && !tool->isBusy() && tool->actors_.empty() && main.ui->textEdit_Show_CFmes->toPlainText().contains("已取消"), "toolbar cancellation discards completed or partial height result");
    main.StartHeightMeasurement();
    dock->findChild<QPushButton*>("heightCancel")->click(); waitTask(main);
    check(tool->actors_.empty() && !tool->isBusy(), "panel cancel button discards pending result");
    main.StartHeightMeasurement(); tool->stop(); waitTask(main);
    check(!tool->isActive() && tool->actors_.empty(), "closing during calculation leaves no delayed geometry");
    open(); run(); main.DelePointCloud("a_measured");
    check(!tool->isActive() && tool->actors_.empty() && main.m_resultOverlay->isHidden(), "source deletion clears owned height objects and labels");
    main.Slot_ed_undo_Triggered();
    check(main.CloudMap.count("a_measured") && tool->actors_.empty(), "undo restores source without stale height result");
    open(); run(); main.ClearAllPointCloudRGB();
    check(!tool->isActive() && tool->actors_.empty(), "RGB cleanup removes height color layer and restores sources");
    open(); run(); main.ui->action_ed_cleangall->trigger();
    check(tool->actors_.empty() && !tool->isActive() && main.m_resultOverlay->isHidden(), "clear all leaves no height geometry or Qt annotation");
    open(); check(!tool->isActive(), "empty scene cannot open height measurement");
    auto invalid = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*item.measured);
    (*invalid)[0].z = std::numeric_limits<float>::quiet_NaN(); invalid->is_dense = false;
    main.AddPointCloud("a_invalid", invalid, ColorManager(255,255,255));
    main.AddPointCloud("b_reference", item.reference, ColorManager(90,150,210));
    open(); run();
    check(tool->actors_.empty() && main.ui->textEdit_Show_CFmes->toPlainText().contains("无效坐标"), "nonfinite input gives concise failure and no partial display");
    main.ClearAllPointCloud();
    auto tilted = heightCase(2);
    main.AddPointCloud("a_measured", tilted.measured, ColorManager(220,230,235));
    main.AddPointCloud("b_reference", tilted.reference, ColorManager(90,150,210));
    open(); run();
    MeasureHeight tiltedBaseline(tilted.measured, tilted.reference); std::vector<double> averages;
    tiltedBaseline.measureIterative(averages,10,5);
    const auto coefficients = tiltedBaseline.GetPlaneCoefficients();
    onPlane = true; planeData = tool->planeActor_->GetMapper()->GetInput();
    for (vtkIdType i=0;i<planeData->GetNumberOfPoints();++i) {
        double point[3]; planeData->GetPoint(i,point);
        onPlane &= std::abs(coefficients->values[0]*point[0]+coefficients->values[1]*point[1]
                          +coefficients->values[2]*point[2]+coefficients->values[3]) < 1e-5;
    }
    check(onPlane, "displayed tilted reference rectangle satisfies fitted plane equation");
    maxLine = tool->actors_[4]->GetMapper()->GetInput(); maxLine->GetPoint(0,p); maxLine->GetPoint(1,q);
    check(std::abs(std::hypot(p[0]-q[0],p[1]-q[1],p[2]-q[2])-tiltedBaseline.GetMaxDistance()) < 1e-5,
          "tilted dimension measures normal distance rather than vertical Z difference");
    capture("tilted_result");
    main.ClearAllPointCloud();
    auto shortReference = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    shortReference->emplace_back(0,0,0); shortReference->emplace_back(1,0,0);
    main.AddPointCloud("a_measured", item.measured, ColorManager(220,230,235));
    main.AddPointCloud("b_reference", shortReference, ColorManager(90,150,210));
    open(); run();
    check(tool->actors_.empty() && main.ui->textEdit_Show_CFmes->toPlainText().contains("不可测"), "failed reference fitting leaves sources visible and shows concise failure");
    main.close(); pump();
    check(!tool->isActive(), "main window close releases active height session");
    std::cout << "pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
