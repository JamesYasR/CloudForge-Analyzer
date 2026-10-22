#include "CloudForgeAnalyzer.h"
#include "Basic/MeasurementPresentation.h"
#include "Basic/ViewportResultOverlay.h"
#include "Basic/WeldPreparationOverlay.h"
#include "Dialog/ParamDialogMeasureWeldHeight.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QCheckBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <vtkCamera.h>
#include <vtkDataSet.h>
#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <iostream>
#include <fstream>
#include <iomanip>

static int passed=0,failed=0;
static void check(bool ok,const char* text) { ok ? ++passed:++failed;std::cout<<(ok ? "[PASS] ":"[FAIL] ")<<text<<std::endl; }
static void pump(int ms=80) { QElapsedTimer timer;timer.start();do { QApplication::processEvents();QThread::msleep(2); } while (timer.elapsed()<ms); }
static void waitTask(CloudForgeAnalyzer& main) { QElapsedTimer timer;timer.start();while (main.IsTaskRunning() && timer.elapsed()<20000) pump(15);pump();check(!main.IsTaskRunning(),"worker finishes while desktop remains responsive"); }
static void capture(QWidget& widget,const QString& name) { pump();check(widget.grab().save("analysis/measurement_visuals/"+name+qEnvironmentVariable("MEASUREMENT_SCALE_SUFFIX")+".png"),"desktop evidence saved"); }
static void weldTest()
{
    {
        QWidget view;auto renderer=vtkSmartPointer<vtkRenderer>::New();ViewportResultOverlay results(&view);
        MeasurementPresentation presentation(&view,renderer,&results);
        check(!renderer->IsActiveCameraCreated(),"shared display construction preserves lazy camera creation");
    }
    auto base=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(),weld=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int x=-35;x<=35;++x) for (int y=-25;y<=25;++y) base->emplace_back(x,y,0);
    for (int x=-20;x<=20;++x) for (int y=-4;y<=4;++y) weld->emplace_back(x,y,3+2*std::exp(-.02*(x*x+y*y)));
    MeasureWeldHeight measure;measure.setWeldCloud(weld);measure.setBaseCloud(base);measure.setSearchRadius(20);measure.setRegionSize(10);measure.setRansacThreshold(.5);measure.setMinNeighbors(15);measure.setVerbose(false);
    const auto result=measure.evaluate();const auto heatmap=measure.getHeatMapCloud();double maximum=measure.getMaxAbsHeight();
    check(result.valid_points>0 && std::abs(result.highest_value-5)<1e-5,"unchanged height algorithm measures known 5 mm peak");
    const auto original=*heatmap;
    CloudForgeAnalyzer main;main.resize(1500,950);main.show();pump();main.ClearAllPointCloud();
    main.AddPointCloud("wall",base,ColorManager(160,180,200));main.AddPointCloud("weld",weld,ColorManager(255,255,255));
    auto view=main.ui->winOfAnalyzer;auto renderer=view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto sphere=vtkSmartPointer<vtkSphereSource>::New();sphere->SetCenter(0,0,6);sphere->SetRadius(14);sphere->SetThetaResolution(32);sphere->SetPhiResolution(24);sphere->Update();
    auto mapper=vtkSmartPointer<vtkPolyDataMapper>::New();mapper->SetInputData(sphere->GetOutput());auto blocker=vtkSmartPointer<vtkActor>::New();blocker->SetMapper(mapper);main.AddActors("reference_geometry",blocker);
    auto camera=renderer->GetActiveCamera();camera->SetPosition(0,-55,70);camera->SetFocalPoint(0,0,0);camera->SetViewUp(0,0,1);renderer->ResetCameraClippingRange();view->renderWindow()->Render();
    double pose[3];camera->GetPosition(pose);auto style=view->interactor()->GetInteractorStyle();
    int stage=0;QTimer dialogs;
    QObject::connect(&dialogs,&QTimer::timeout,[&]() {
        auto modal=qobject_cast<QDialog*>(QApplication::activeModalWidget());if (!modal) return;
        auto title=modal->windowTitle();
        if (title.contains("选择焊缝")) { for (auto box:modal->findChildren<QCheckBox*>()) box->setChecked(box->text()=="weld");stage=1; }
        else if (title.contains("选择圆柱壁面")) { for (auto box:modal->findChildren<QCheckBox*>()) box->setChecked(box->text()=="wall");stage=2; }
        else if (title=="焊缝高度测量") stage=3;
        else { modal->reject();return; }
        for (auto button:modal->findChildren<QPushButton*>()) if (button->text()=="确定") { button->click();break; }
    });
    dialogs.start(25);main.ui->measure_weldheight->trigger();dialogs.stop();waitTask(main);
    check(stage==3,"original weld then wall then parameter workflow remains intact");
    auto display=main.m_presentation.get();auto& group=display->groups_.at("weld_height");
    check(group.heatmaps.size()==1 && group.geometry.empty() && group.annotations.size()==1,"height result uses heatmap and hollow Qt callout without masking sphere");
    check(group.annotations[0].anchor[2]==result.highest_point.z && group.annotations[0].title.contains("5.000"),"highest point label attaches to true computed peak");
    check(blocker->GetProperty()->GetRepresentation()==VTK_WIREFRAME && blocker->GetProperty()->GetOpacity()<.2,"opaque reference geometry no longer covers peak");
    check(!main.viewer->getCloudActorMap()->at("weld").actor->GetVisibility(),"heatmap replaces coincident raw cloud display");
    check(display->legend_->isVisible() && display->legend_->entries_.size()==1 && display->legend_->entries_[0].minimum==-maximum && display->legend_->entries_[0].maximum==maximum,"named signed color scale matches heatmap range");
    check(display->legend_->entries_[0].colors==ViewportColorLegend::signedHeight(),"signed height legend includes cyan and yellow like point colors");
    pump();const auto legendImage=display->legend_->grab().toImage();const double dpr=legendImage.devicePixelRatio();
    const auto red=legendImage.pixelColor(qRound(25*dpr),qRound(78*dpr));
    const auto blue=legendImage.pixelColor(qRound(25*dpr),qRound((display->legend_->height()-28)*dpr));
    const auto green=legendImage.pixelColor(qRound(25*dpr),qRound((74+(display->legend_->height()-98)/2.0)*dpr));
    check(red.red()>220 && blue.blue()>220 && green.green()>220 && green.red()<30 && green.blue()<30,"rendered legend gradient displays red green and blue pixels");

    check(std::equal(pose,pose+3,camera->GetPosition()) && style==view->interactor()->GetInteractorStyle(),"height display preserves camera and interactor");
    check(!display->annotations_->filledMarkers_ && display->annotations_->testAttribute(Qt::WA_TransparentForMouseEvents),"peak marker stays hollow and annotation does not capture mouse");
    capture(main,"weld_height");
    main.cleanWeldMeasureVisuals();main.VisualizeWeldHeight("weld",result,heatmap,maximum);main.VisualizeWeldHeight("second",result,heatmap,maximum);pump();
    check(display->groups_.at("weld_height").annotations.size()==2 && display->legend_->selector_->isVisible(),"multiple weld results retain individually named scales");
    main.ui->action_ed_clean2DActor->trigger();pump();
    check(display->annotations_->isHidden() && display->legend_->isHidden() && display->groups_.at("weld_height").heatmaps.size()==2,"2D cleanup clears Qt annotations and scales but retains heatmap");
    main.ui->action_ed_cleanRGB->trigger();pump();
    check(display->groups_.empty() && main.viewer->getCloudActorMap()->at("weld").actor->GetVisibility(),"RGB cleanup removes result and restores raw cloud");
    check(blocker->GetProperty()->GetRepresentation()==VTK_SURFACE && blocker->GetProperty()->GetOpacity()==1,"cleanup restores original geometry properties");
    bool unchanged=heatmap->size()==original.size();for (size_t i=0;i<heatmap->size();++i) unchanged &= (*heatmap)[i].rgba==original[i].rgba && (*heatmap)[i].x==original[i].x && (*heatmap)[i].z==original[i].z;
    check(unchanged,"presentation preserves calculated point positions and colors bitwise");
    main.VisualizeWeldHeight("weld",result,heatmap,maximum);main.ClearAllPointCloud();pump();check(display->groups_.empty() && display->legend_->isHidden(),"cloud removal clears owned Qt and VTK results together");
    std::ofstream out("analysis/measurement_visuals/weld_values.txt");out<<std::setprecision(17)<<result.highest_value<<' '<<result.highest_point.x<<' '<<result.highest_point.y<<' '<<result.highest_point.z<<' '<<result.valid_points<<'\n';
    main.close();pump();
}
int main(int argc,char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
    weldTest();std::cout<<"pass="<<passed<<" fail="<<failed<<std::endl;return failed ? 1:0;
}
