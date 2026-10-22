#include "CloudForgeAnalyzer.h"
#include "Basic/WeldPreparationOverlay.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QElapsedTimer>
#include <QThread>
#include <QMouseEvent>
#include <vtkCamera.h>
#include <vtkCallbackCommand.h>
#include <vtkDataSet.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

static int passed = 0, failed = 0;
static void check(bool condition, const char* name)
{
    condition ? ++passed : ++failed;
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << std::endl;
}

static void settle(int ms = 150)
{
    QElapsedTimer timer;
    timer.start();
    do { QApplication::processEvents(); QThread::msleep(2); } while (timer.elapsed() < ms);
}

static void mouse(QWidget* view, QEvent::Type type, QPointF point, Qt::MouseButton button, Qt::MouseButtons buttons)
{
    QMouseEvent event(type, point, QPointF(view->mapToGlobal(point.toPoint())), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(view, &event);
}

static bool samePoint(vtkDataSet* data, vtkIdType index, const Eigen::Vector3d& expected)
{
    double actual[3];
    data->GetPoint(index, actual);
    for (int i = 0; i < 3; ++i) if (actual[i] != static_cast<float>(expected[i])) return false;
    return true;
}

int main(int argc, char** argv)
{
    const std::string root = "analysis/weld_preparation/samples_realistic/ring_g5_de+3/";
    auto nearCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto farCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(root + "near.pcd", *nearCloud) < 0
        || pcl::io::loadPCDFile(root + "far.pcd", *farCloud) < 0) return 1;
    const Eigen::Vector3d axis = Eigen::Vector3d(.37, .48, .79).normalized();
    const Eigen::Vector3d radial = axis.cross(Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d along = axis.cross(radial).normalized();
    MeasureWeldPreparation measure;
    MeasureWeldPreparation::Params params;
    params.design_radius = 1940;
    params.metric = MeasureWeldPreparation::Metric::Both;
    params.gap_direction = MeasureWeldPreparation::GapDirection::Axial;
    measure.setParams(params);
    measure.setNearCloud(nearCloud, "near");
    measure.setFarCloud(farCloud, "far");
    measure.setFixedReferenceAxis(Eigen::Vector3d::Zero(), axis);
    measure.setRoleOverride(MeasureWeldPreparation::Role::Near, MeasureWeldPreparation::Role::Far, "visual review");
    const auto result = measure.evaluate();
    std::cout << result.report << std::endl;
    if (!result.maximum.ok || !result.maximum_step.ok) return 2;
    std::ofstream("build/weld_visual_after_report.txt") << result.report;

    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);
    {
        QWidget unusedView;
        auto unusedRenderer = vtkSmartPointer<vtkRenderer>::New();
        { WeldPreparationOverlay overlay(&unusedView, unusedRenderer);
          check(!unusedRenderer->IsActiveCameraCreated(), "overlay construction preserves lazy camera creation"); }
        check(!unusedRenderer->IsActiveCameraCreated(), "hidden overlay teardown does not create a camera");
    }
    CloudForgeAnalyzer main;
    main.resize(1450, 950);
    main.show();
    settle();
    main.ClearAllPointCloud();
    main.AddPointCloud("near", nearCloud, ColorManager(235, 235, 235));
    main.AddPointCloud("far", farCloud, ColorManager(235, 235, 235));
    auto view = main.ui->winOfAnalyzer;
    auto renderer = view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto camera = renderer->GetActiveCamera();
    const Eigen::Vector3d center = radial * 1940 + axis * 45;
    const char* suffix = std::getenv("WELD_CAPTURE_SUFFIX");
    auto capture = [&](const char* name, const Eigen::Vector3d& focus, const Eigen::Vector3d& direction, double distance) {
        const Eigen::Vector3d position = focus + direction.normalized() * distance;
        camera->SetPosition(position.data());
        camera->SetFocalPoint(focus.data());
        camera->SetViewUp(along.data());
        renderer->ResetCameraClippingRange();
        view->renderWindow()->Render();
        settle();
        const bool saved = main.grab().save(QString("analysis/weld_preparation/ui_evidence/%1%2.png").arg(name, suffix ? suffix : "_updated"));
        check(saved, name);
        return saved;
    };
    capture("sample_unannotated", center, radial, 950);
    main.Update_CFmes(result.report);
    double position[3]; camera->GetPosition(position);
    auto style = view->interactor()->GetInteractorStyle();
    main.VisualizeWeldPreparation(result, measure.frame());
    auto overlay = main.m_weldPrepOverlay.get();
    check(overlay->isVisible() && overlay->annotations_.size() == 2, "two labelled extrema in existing viewport");
    check(overlay->annotations_[0].title == "最大轴向间隙 5.373 mm"
          && overlay->annotations_[1].title == "最大径向阶差 3.331 mm"
          && overlay->annotations_[0].detail == "整体中值 5.204 mm"
          && overlay->annotations_[1].detail == "整体中值 2.999 mm", "Chinese quantity names and original numeric results");
    check(std::equal(position, position+3, camera->GetPosition()) && style == view->interactor()->GetInteractorStyle(),
          "display preserves camera and VTK interactor");
    check(overlay->testAttribute(Qt::WA_TransparentForMouseEvents), "annotations do not intercept mouse operations");
    bool edgesPreserved = true;
    for (const auto& item : {std::make_pair(&result.near_edge, "near"), std::make_pair(&result.far_edge, "far")}) {
        for (size_t i = 0; i < item.first->refined.size(); ++i) {
            const auto& polyline = item.first->refined[i];
            if (polyline.size() < 2) continue;
            auto actor = main.ActorMap.at("weldprep_" + std::string(item.second) + "_edge_" + std::to_string(i));
            auto data = actor->GetMapper()->GetInput();
            if (data->GetNumberOfPoints() != static_cast<vtkIdType>(polyline.size())) edgesPreserved = false;
            else for (size_t j = 0; j < polyline.size(); ++j)
                edgesPreserved &= samePoint(data,j,measure.frame().unproject(polyline[j].x(),polyline[j].y(),0));
        }
    }
    check(edgesPreserved, "edge geometry preserves all original projected coordinates");
    auto gap = main.ActorMap.at("weldprep_gap_arc")->GetMapper()->GetInput();
    bool gapPreserved = gap->GetNumberOfPoints() == 25;
    for (int i = 0; i <= 24; ++i) {
        const double f = double(i)/24;
        gapPreserved &= samePoint(gap,i,measure.frame().unproject(
            result.maximum.near_a + f*(result.maximum.far_a-result.maximum.near_a),
            result.maximum.near_s + f*(result.maximum.far_s-result.maximum.near_s),0));
    }
    check(gapPreserved, "gap dimension path preserves original 25 coordinates");
    const auto stepMatch = std::find_if(result.matches.begin(),result.matches.end(),[&](const auto& m) {
        return m.valid && m.near_index == result.maximum_step.near_index;
    });
    const auto& step = main.LineMap.at("weldprep_step_radial");
    const Eigen::Vector3f expectedStart = measure.frame().unproject(stepMatch->near_a,stepMatch->near_s,stepMatch->far_e).cast<float>();
    const Eigen::Vector3f expectedEnd = measure.frame().unproject(stepMatch->near_a,stepMatch->near_s,stepMatch->near_e).cast<float>();
    check((step.start.getVector3fMap()-expectedStart).norm() == 0 && (step.end.getVector3fMap()-expectedEnd).norm() == 0,
          "radial dimension preserves original material endpoints");
    capture("sample_front", center, radial, 950);
    capture("sample_oblique", center, radial + axis * .55, 950);
    capture("sample_detail", result.maximum.near_xyz, radial + axis * .35, 260);
    auto labels = overlay->annotations_;
    labels[1].anchor = labels[0].anchor;
    overlay->setAnnotations(labels);
    capture("layout_coincident_anchors", result.maximum.near_xyz, radial, 650);
    main.VisualizeWeldPreparation(result, measure.frame());
    const double edgeOffset = .8 * 260 * std::tan(camera->GetViewAngle()*std::acos(-1.0)/360) * renderer->GetTiledAspectRatio();
    capture("sample_edge", result.maximum.near_xyz + axis*edgeOffset, radial, 260);
    QPointF marker;
    const bool projected = overlay->project(overlay->annotations_[0].anchor, marker);
    const auto image = overlay->grab().toImage();
    const double pixelRatio = overlay->devicePixelRatioF();
    const auto markerColor = projected ? image.pixelColor(qRound(marker.x()*pixelRatio),qRound(marker.y()*pixelRatio)) : QColor();
    check(projected && markerColor.red() > 235 && markerColor.green() > 235 && markerColor.blue() > 235,
          "edge label leaves the extreme marker visible");
    for (const auto& name : {"near","far"})
        main.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,.12,.22,.29,name);
    capture("sample_dark", center, radial, 950);
    main.resize(1030,760); settle();
    capture("sample_small", center, radial, 1100);
    check(overlay->geometry() == view->rect(), "overlay follows viewport resizing");
    const QPointF press(view->width()*.45,view->height()*.55);
    camera->GetPosition(position);
    mouse(view,QEvent::MouseButtonPress,press,Qt::LeftButton,Qt::LeftButton);
    mouse(view,QEvent::MouseMove,press+QPointF(60,35),Qt::NoButton,Qt::LeftButton);
    mouse(view,QEvent::MouseButtonRelease,press+QPointF(60,35),Qt::LeftButton,Qt::NoButton);
    settle();
    check(!std::equal(position,position+3,camera->GetPosition()) && style == view->interactor()->GetInteractorStyle(),
          "actual mouse drag rotates the original camera through the overlay");
    int renders = 0;
    auto callback = vtkSmartPointer<vtkCallbackCommand>::New();
    callback->SetClientData(&renders);
    callback->SetCallback([](vtkObject*,unsigned long,void* data,void*) { ++*static_cast<int*>(data); });
    const auto observer = renderer->AddObserver(vtkCommand::EndEvent,callback);
    settle(350);
    renderer->RemoveObserver(observer);
    check(renders < 5,"idle annotation repaint does not trigger a render loop");
    const auto actorCount = main.m_weldPrepActorIds.size();
    for (int i = 0; i < 4; ++i) main.VisualizeWeldPreparation(result, measure.frame());
    check(main.m_weldPrepActorIds.size() == actorCount && overlay->annotations_.size() == 2,
          "repeated visualization replaces annotations without accumulating actors");
    main.ui->action_ed_cleangeo->trigger(); settle();
    check(!overlay->isVisible() && main.m_weldPrepActorIds.empty() && main.m_weldPrepLineIds.empty(),
          "clear geometry removes all weld annotations and lines");
    check(main.CloudMap.at("near") == nearCloud && main.CloudMap.at("far") == farCloud,
          "display and cleanup preserve both original source clouds");
    main.VisualizeWeldPreparation(result, measure.frame());
    main.VisualizeWeldPreparation(MeasureWeldPreparation::Result(), measure.frame());
    check(!overlay->isVisible() && main.m_weldPrepActorIds.empty(),"unmeasurable result clears previous visualization");
    main.VisualizeWeldPreparation(result, measure.frame());
    main.DelePointCloud("near");
    check(!overlay->isVisible() && main.m_weldPrepActorIds.empty(),"removing source cloud clears its annotations");
    main.ui->action_ed_undo->trigger(); settle();
    check(!overlay->isVisible() && main.CloudMap.count("near"),"undo restores cloud without stale measurement labels");
    main.VisualizeWeldPreparation(result, measure.frame());
    main.ui->action_ed_cleangall->trigger(); settle();
    check(!overlay->isVisible() && main.m_weldPrepActorIds.empty() && main.CloudMap.empty(),"clear all leaves no stale labels or actors");
    std::cout << "[VISUAL] screenshot capture complete; input near=" << nearCloud->size()
              << " far=" << farCloud->size() << " DPR=" << view->devicePixelRatioF() << std::endl;
    main.close();
    settle();
    std::cout << "[RESULT] pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
