#include "CloudForgeAnalyzer.h"
#include "Basic/MeasurementPresentation.h"
#include "Basic/ArcMeasurementTool.h"
#include "Basic/ViewportPointPicker.h"
#include <QDockWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Basic/ViewportResultOverlay.h"
#include "Basic/WeldPreparationOverlay.h"
#include "Dialog/ParamDialogMeasureWeldHeight.h"
#include "Dialog/ParamDialog_Pothole.h"
#include <QToolButton>
#include <QLineEdit>
#include <QFrame>
#include <pcl/io/pcd_io.h>
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
static void pitsTest()
{
    ParamDialog_Pothole dialog;dialog.show();pump();
    const QVector<QString> defaults={"0","0","200","0","20","90","0.35","1","0"};
    check(dialog.getParams()==defaults,"hidden pothole parameters preserve all nine defaults and indices");
    int visible=0;for (auto edit:dialog.paramEdits) visible+=edit->isVisible();check(visible==2,"pothole form initially shows only threshold and baseline window");
    capture(dialog,"pothole_parameters");dialog.findChild<QToolButton*>("potholeAdvanced")->click();pump();
    visible=0;for (auto edit:dialog.paramEdits) visible+=edit->isVisible();check(visible==4 && dialog.getParams()==defaults,"advanced settings expose clustering controls without changing defaults");dialog.reject();
    auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();check(pcl::io::loadPCDFile("PCDfiles/pothole_dent_form.pcd",*cloud)==0,"four-pit reference cloud loaded");
    auto cylinder=pcl::make_shared<pcl::ModelCoefficients>();cylinder->values.resize(7);std::ifstream axis("analysis/cylinder_patch/out_multipit/axis_form.txt");for(int i=0;i<6;++i) axis>>cylinder->values[i];cylinder->values[6]=1940;
    MeasurePothole measure;measure.setInputCloud(cloud);measure.setCylinder(cylinder);measure.setDistanceThreshold(0);measure.setClusterTolerance(5);measure.setMinClusterSize(60);
    measure.setTrendMode(0);measure.setMaxAreaFraction(.2);measure.setUseLocalBaseline(true);measure.setLocalWindow(90);measure.setLocalThreshold(.35);measure.setBoundaryExclude(true);measure.setHeatMapField(0);measure.setVerbose(false);
    const auto result=measure.evaluate();const auto snapshot=result;
    check(result.pit_count==4 && result.pits.size()==4 && result.valid,"four-pit baseline still detects all four pits");
    check(std::abs(result.local_max_depth-1.54914587)<5e-9 && std::abs(result.max_depth-3.14428711)<5e-9,"local and ideal-cylinder depth values match frozen baseline");
    auto heatmap=measure.getHeatMapCloud();std::vector<uint32_t> colors;for(size_t i=0;i<heatmap->size();i+=97) colors.push_back((*heatmap)[i].rgba);
    CloudForgeAnalyzer main;main.resize(1550,1000);main.show();pump();main.ClearAllPointCloud();main.AddPointCloud("specimen",cloud,ColorManager(210,220,230));main.cylinderResultsMap["reference"]=cylinder;
    auto view=main.ui->winOfAnalyzer;auto renderer=view->renderWindow()->GetRenderers()->GetFirstRenderer();const int initialLayers=view->renderWindow()->GetNumberOfLayers();
    pcl::PointXYZ lo,hi;pcl::getMinMax3D(*cloud,lo,hi);const Eigen::Vector3d center=((lo.getVector3fMap()+hi.getVector3fMap())*.5f).cast<double>();
    const Eigen::Vector3d origin(cylinder->values[0],cylinder->values[1],cylinder->values[2]),u=Eigen::Vector3d(cylinder->values[3],cylinder->values[4],cylinder->values[5]).normalized();
    const Eigen::Vector3d delta=center-origin,radial=(delta-u*delta.dot(u)).normalized(),eye=center+500*radial;
    auto camera=renderer->GetActiveCamera();camera->SetPosition(eye.data());camera->SetFocalPoint(center.data());camera->SetViewUp(u.data());camera->ParallelProjectionOn();camera->SetParallelScale(175);renderer->ResetCameraClippingRange();view->renderWindow()->Render();
    double pose[3];camera->GetPosition(pose);auto style=view->interactor()->GetInteractorStyle();main.VisualizePothole(measure,result,"specimen");pump();
    auto display=main.m_presentation.get();auto& group=display->groups_.at("pothole");
    check(group.annotations.size()==5 && group.foreground.size()==8,"four pit callouts and global maximum pair with haloed ellipse outlines");
    check(group.heatmaps.size()==1 && !main.viewer->getCloudActorMap()->at("specimen").actor->GetVisibility(),"pothole heatmap replaces coincident raw display");
    check(main.CloudMap.count(main.m_potholeOutputName) && !main.viewer->getCloudActorMap()->at(main.m_potholeOutputName).actor->GetVisibility(),"extracted pit cloud remains available without covering heatmap");
    check(display->foreground_ && display->foreground_->GetActiveCamera()==camera && !display->foreground_->GetPreserveDepthBuffer(),"contours remain visible using unchanged projected geometry");
    bool ellipses=true;for(size_t i=0;i<result.pits.size();++i) {
        auto data=group.foreground[i*2+1]->GetMapper()->GetInput();ellipses &= data->GetNumberOfPoints()==static_cast<vtkIdType>(result.pits[i].ellipse_points.size());
        for(vtkIdType j=0;j<data->GetNumberOfPoints();++j) { double p[3];data->GetPoint(j,p);ellipses &= (Eigen::Vector3d(p[0],p[1],p[2])-result.pits[i].ellipse_points[j].cast<double>()).norm()<1e-5; }
    }
    check(ellipses,"displayed ellipses preserve original fitted coordinates");
    pump();const auto image=view->grab().toImage();const auto size=view->renderWindow()->GetSize();int colored=0;
    for(size_t side=0;side<result.pits.size();++side) {
        const auto expected=result.pits[side].is_main ? QColor("#ff73d3"):QColor("#ffd166");bool visibleColor=false;
        for(size_t j=0;j<result.pits[side].ellipse_points.size();j+=5) {
            const auto& p=result.pits[side].ellipse_points[j];renderer->SetWorldPoint(p.x(),p.y(),p.z(),1);renderer->WorldToDisplay();const auto screen=renderer->GetDisplayPoint();
            const int x=qRound(screen[0]*image.width()/size[0]),y=qRound((1-screen[1]/size[1])*image.height());
            for(int dx=-3;dx<=3;++dx) for(int dy=-3;dy<=3;++dy) if(image.rect().contains(x+dx,y+dy)) { const auto pixel=image.pixelColor(x+dx,y+dy);visibleColor |= std::abs(pixel.red()-expected.red())+std::abs(pixel.green()-expected.green())+std::abs(pixel.blue()-expected.blue())<35; }
        }
        colored+=visibleColor;
    }
    check(colored==4,"each fitted ellipse has visible colored pixels on the desktop");
    check(!main.ui->textEdit_Show_CFmes->toPlainText().contains("窗口") && main.ui->textEdit_Show_CFmes->toPlainText().contains("到理想柱面最大距离"),"pothole report contains results without internal tuning parameters");

    check(std::equal(pose,pose+3,camera->GetPosition()) && style==view->interactor()->GetInteractorStyle(),"pothole presentation preserves camera and interaction");capture(main,"pothole_result");
    main.VisualizePothole(measure,result,"specimen");pump();check(display->groups_.at("pothole").foreground.size()==8 && main.CloudMap.size()==2,"repeated pit results replace contours and extracted output without accumulating");
    main.ui->action_ed_clean2DActor->trigger();pump();check(display->annotations_->isHidden() && display->groups_.at("pothole").foreground.size()==8,"2D cleanup removes pit callouts while retaining fitted outlines");
    main.ui->action_ed_cleanRGB->trigger();pump();check(display->groups_.empty() && !display->foreground_ && view->renderWindow()->GetNumberOfLayers()==initialLayers,"RGB cleanup releases heatmap contours and temporary renderer");
    check(main.viewer->getCloudActorMap()->at("specimen").actor->GetVisibility(),"pit cleanup restores source visibility");
    bool unchanged=result.max_depth==snapshot.max_depth && result.local_max_depth==snapshot.local_max_depth && result.pits.size()==snapshot.pits.size();size_t k=0;for(size_t i=0;i<heatmap->size();i+=97) unchanged &= (*heatmap)[i].rgba==colors[k++];
    check(unchanged,"pit measurements and heatmap colors remain unchanged by UI");
    main.close();pump();
}

static void clickPoint(QWidget* view,vtkRenderer* renderer,const Eigen::Vector3d& point)
{
    renderer->SetWorldPoint(point.x(),point.y(),point.z(),1);renderer->WorldToDisplay();auto p=renderer->GetDisplayPoint();
    auto window=renderer->GetRenderWindow();const auto size=window->GetSize();
    const QPointF pos(p[0]*view->width()/size[0],(1-p[1]/size[1])*view->height());
    QMouseEvent down(QEvent::MouseButtonPress,pos,view->mapToGlobal(pos.toPoint()),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent up(QEvent::MouseButtonRelease,pos,view->mapToGlobal(pos.toPoint()),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(view,&down);QApplication::sendEvent(view,&up);pump(140);
}
static void arcsTest()
{
    auto cylinderCloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for(int z=-5;z<=5;++z) for(int i=0;i<=120;++i) { const double a=-.6+i*.01;cylinderCloud->emplace_back(100*std::cos(a),100*std::sin(a),z); }
    auto cylinder=pcl::make_shared<pcl::ModelCoefficients>();cylinder->values={0,0,0,0,0,1,100};
    MeasureArc::ArcParams params;params.method=MeasureArc::BSPLINE_LSQ;
    MeasureArc baseline(cylinderCloud,cylinder,nullptr,params);baseline.compute();
    check(baseline.success && std::abs(baseline.arcLength-120)<.3,"unchanged B spline algorithm measures analytic 120 mm arc");
    const auto sourcePositions=baseline.m_fittedPoints2D;
    params.method=MeasureArc::CARDINAL_SPLINE;MeasureArc cardinal(cylinderCloud,cylinder,nullptr,params);cardinal.compute();
    check(cardinal.success && cardinal.arcLength>119 && cardinal.arcLength<130,"Cardinal calculation remains available with original defaults");
    CloudForgeAnalyzer main;main.resize(1500,950);main.show();pump();main.ClearAllPointCloud();main.AddPointCloud("cylinder_surface",cylinderCloud,ColorManager(210,220,230));main.cylinderResultsMap["C001"]=cylinder;
    auto view=main.ui->winOfAnalyzer;auto renderer=view->renderWindow()->GetRenderers()->GetFirstRenderer();auto camera=renderer->GetActiveCamera();auto style=view->interactor()->GetInteractorStyle();
    camera->SetPosition(400,0,60);camera->SetFocalPoint(95,0,0);camera->SetViewUp(0,0,1);camera->ParallelProjectionOn();camera->SetParallelScale(70);renderer->ResetCameraClippingRange();view->renderWindow()->Render();
    main.ui->measure_cylinder->trigger();pump();camera->SetParallelScale(std::max(70.0,70.0*view->height()/std::max(1,view->width())));renderer->ResetCameraClippingRange();view->renderWindow()->Render();auto tool=main.m_arcTool.get();
    check(tool->isActive() && !tool->isGeodesic() && tool->dock_->isVisible() && !QApplication::activeModalWidget(),"cylinder settings open as native dock without nested dialogs");
    MeasureArc::ArcParams selected;double radius=0;check(tool->parameters(selected,radius) && selected.method==MeasureArc::BSPLINE_LSQ && selected.downsampleTargetSize==30 && selected.bsplineControlPoints==15 && radius==0,"initial method and all original defaults are retained");
    check(tool->advanced_->isVisible() && tool->fields_[7]->isHidden(),"all cylinder parameters are visible without collapsed settings");pump();check(tool->fields_[4]->isVisible() && !tool->fields_[2]->isVisible(),"all active spline parameters are directly accessible");
    tool->method_->setCurrentIndex(0);check(tool->fields_[2]->isVisible() && !tool->fields_[4]->isVisible(),"Cardinal settings follow native method selector");tool->method_->setCurrentIndex(1);
    tool->measure_->click();waitTask(main);auto display=main.m_presentation.get();
    check(display->groups_.count("cylinder_arc") && !tool->isBusy(),"middle-section measurement finishes through new dock");
    if (!display->groups_.count("cylinder_arc")) { main.close();return; }
    auto& group=display->groups_.at("cylinder_arc");
    check(group.foreground.size()==2 && group.annotations.size()==1 && group.annotations[0].endpoints.size()==2,"arc has halo, label and endpoint indicators");
    check(group.annotations[0].title.contains(QString::number(baseline.arcLength,'f',3)),"displayed arc value matches original computation");
    check(display->foreground_->GetActiveCamera()==camera && !display->foreground_->GetPreserveDepthBuffer() && group.foreground.back()->GetProperty()->GetLineWidth()>=5*view->devicePixelRatioF(),"curve stays visible over geometry and scales with desktop DPI");
    double pose[3];camera->GetPosition(pose);main.VisualizeArcMeasurement(baseline.getVisualizationActor(),baseline.arcLength);pump();
    check(sourcePositions==baseline.m_fittedPoints2D && std::equal(pose,pose+3,camera->GetPosition()) && style==view->interactor()->GetInteractorStyle(),"curve styling preserves fitted geometry camera and interaction");
    pump();const auto arcImage=view->grab().toImage();auto arcData=display->groups_.at("cylinder_arc").foreground.back()->GetMapper()->GetInput();const auto rwsize=view->renderWindow()->GetSize();int bright=0;
    for(vtkIdType i=10;i<80;i+=5) { double p[3];arcData->GetPoint(i,p);renderer->SetWorldPoint(p[0],p[1],p[2],1);renderer->WorldToDisplay();auto d=renderer->GetDisplayPoint();const int x=qRound(d[0]*arcImage.width()/rwsize[0]),y=qRound((1-d[1]/rwsize[1])*arcImage.height());bool found=false;for(int dx=-3;dx<=3;++dx) for(int dy=-3;dy<=3;++dy) if(arcImage.rect().contains(x+dx,y+dy)) { auto c=arcImage.pixelColor(x+dx,y+dy);found |= c.green()>185 && c.blue()>220 && c.red()<150; }bright+=found; }
    check(bright>=12,"bright cyan curve remains visible along its actual projected path");capture(main,"arc_cylinder");tool->dock_->setFloating(true);pump();check(tool->dock_->isFloating(),"arc settings can float using native dock behavior");tool->dock_->setFloating(false);pump();
    tool->position_->setCurrentIndex(1);tool->measure_->click();pump();check(tool->isPicking() && main.m_pointPicker->isSelecting(),"manual section uses the shared distance point picker");
    clickPoint(view,renderer,Eigen::Vector3d(100,0,0));waitTask(main);check(!tool->isBusy() && !main.m_pointPicker->isSelecting() && display->groups_.count("cylinder_arc"),"single point section selection measures without changing interactor");
    tool->measure_->click();pump();QKeyEvent escape(QEvent::KeyPress,Qt::Key_Escape,Qt::NoModifier);QApplication::sendEvent(view,&escape);pump();check(!tool->isBusy() && tool->isActive(),"Escape cancels point selection and restores usable settings");
    tool->stop();pump();check(display->groups_.count("cylinder_arc") && !tool->isActive(),"closing panel keeps completed arc result");main.cleanArcMeasurement();pump();check(!display->foreground_ && !display->groups_.count("cylinder_arc"),"arc cleanup releases foreground renderer and labels");
    main.ui->measure_cylinder->trigger();pump();tool->position_->setCurrentIndex(0);tool->measure_->click();tool->stop();waitTask(main);check(!display->groups_.count("cylinder_arc"),"closing during calculation discards late results");
    tool->start(false,{"first","second"},{"C001","C002"});check(tool->cloud_->currentIndex()==-1 && tool->reference_->currentIndex()==-1 && !tool->measure_->isEnabled(),"multiple inputs require explicit selection without name heuristics");tool->stop();
    main.ClearAllPointCloud();auto plane=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();for(int x=-20;x<=20;++x) for(int y=-10;y<=10;++y) plane->emplace_back(x,y,0);
    main.AddPointCloud("plane",plane,ColorManager(200,220,230));GeodesicArcMeasurer geodesic(plane,3);auto path=geodesic.measureArcLength(pcl::PointXYZ(-20,0,0),pcl::PointXYZ(20,0,0));
    check(path.success && std::abs(path.arc_length-40)<1e-6,"unchanged geodesic algorithm measures 40 mm planar path");
    camera->SetPosition(0,0,100);camera->SetFocalPoint(0,0,0);camera->SetViewUp(0,1,0);camera->SetParallelScale(30);renderer->ResetCameraClippingRange();view->renderWindow()->Render();
    main.ui->measure_geodisic->trigger();pump();camera->SetParallelScale(std::max(30.0,25.0*view->height()/std::max(1,view->width())));renderer->ResetCameraClippingRange();view->renderWindow()->Render();tool->fields_[7]->setText("3");
    check(tool->isGeodesic() && tool->fields_[7]->isVisible() && tool->reference_->isHidden() && tool->advanced_->isHidden(),"geodesic dock shows source and original neighborhood setting only");capture(main,"arc_geodesic_parameters");
    tool->measure_->click();pump();clickPoint(view,renderer,Eigen::Vector3d(-20,0,0));clickPoint(view,renderer,Eigen::Vector3d(20,0,0));waitTask(main);
    check(display->groups_.count("geodesic_arc") && !tool->isBusy(),"two point desktop selection completes geodesic measurement");
    if(display->groups_.count("geodesic_arc")) {
        auto& geo=display->groups_.at("geodesic_arc");check(geo.annotations[0].title.contains("40.000") && geo.foreground.size()==2,"geodesic value has matching bright curve and rectangular callout");
        main.VisualizeGeodesicMeasurement(geodesic,path);auto data=display->groups_.at("geodesic_arc").foreground.back()->GetMapper()->GetInput();bool exact=data->GetNumberOfPoints()==static_cast<vtkIdType>(path.path_points.size());
        for(vtkIdType i=0;i<data->GetNumberOfPoints();++i) { double p[3];data->GetPoint(i,p);exact &= (Eigen::Vector3d(p[0],p[1],p[2])-path.path_points[i].cast<double>()).norm()<1e-5; }check(exact,"all rendered geodesic points retain original path coordinates");capture(main,"arc_geodesic");
        main.ui->action_ed_clean2DActor->trigger();pump();check(display->annotations_->isHidden() && display->groups_.at("geodesic_arc").foreground.size()==2,"2D cleanup removes Qt labels without losing arc curves");
        tool->clear_->click();pump();check(!display->groups_.count("geodesic_arc") && !display->foreground_,"clear result removes owned path and temporary renderer");
    }
    tool->measure_->click();pump();main.ui->action_ed_clean2DActor->trigger();pump();check(!tool->isBusy() && !main.m_pointPicker->isSelecting(),"cleanup during picking cancels selection without locking panel");
    tool->measure_->click();pump();tool->stop();pump();check(!tool->isBusy() && !main.m_pointPicker->isSelecting(),"closing during picking restores normal interaction");
    main.close();pump();
}

int main(int argc,char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
    if (argc>1 && std::string(argv[1])=="--pits") pitsTest();else if(argc>1 && std::string(argv[1])=="--arcs") arcsTest();else weldTest();std::cout<<"pass="<<passed<<" fail="<<failed<<std::endl;return failed ? 1:0;
}
