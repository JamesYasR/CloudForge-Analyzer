#include "Measure/MeasureWeldParallelism.h"
#include "Basic/WeldParallelismTool.h"
#include "Basic/ViewportResultOverlay.h"
#include "CloudForgeAnalyzer.h"
#include "ui_CloudForgeAnalyzer.h"
#include <pcl/io/pcd_io.h>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPushButton>
#include <QThread>
#include <QWheelEvent>
#include <vtkCamera.h>
#include <vtkDataSet.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>

using Measure=MeasureWeldParallelism;
static int passed=0,failed=0;
static void check(bool ok,const std::string& name) { ok ? ++passed:++failed;std::cout<<(ok ? "[PASS] ":"[FAIL] ")<<name<<std::endl; }
static Measure::Cylinder cylinder() { return {{0,0,0},{0,0,1},1940}; }
static Measure::Cloud::Ptr seam(double angle,double offset,double radial=0,double shift=0,double phase=0,double length=160)
{
    CylinderSurfaceFrame frame;frame.init({0,0,0},{0,0,1},1940,phase);
    const double beta=angle*std::acos(-1.0)/180;
    const Eigen::Vector2d along(std::cos(beta),std::sin(beta)),across(-along.y(),along.x());
    auto cloud=pcl::make_shared<Measure::Cloud>();
    for (int i=0;i<=120;++i) for (int j=-3;j<=3;++j) {
        const Eigen::Vector2d p=along*(-length/2+length*i/120)+across*(j*.7)+Eigen::Vector2d(shift,offset);
        const Eigen::Vector3d world=frame.unproject(p.y(),p.x(),radial*std::sin(i*.12));
        cloud->emplace_back(world.x(),world.y(),world.z());
    }
    return cloud;
}
static Measure::Result measure(Measure::Cloud::Ptr a,Measure::Cloud::Ptr b,Measure::Cylinder c=cylinder()) { return Measure(a,b,c).compute(); }
static void pureTests()
{
    for (double angle:{0.,.05,.5,2.,15.,45.,90.}) {
        auto result=measure(seam(0,0),seam(angle,25));
        std::cout<<"[ANGLE] expected="<<angle<<" actual="<<result.angleDegrees<<" success="<<result.success<<" "<<result.message<<std::endl;
        check(result.success && std::abs(result.angleDegrees-angle)<.004,"analytic unfolded angle "+std::to_string(angle));
    }
    auto first=seam(0,0),second=seam(2,25);auto reference=measure(first,second);
    check(reference.success && reference.comparisonLength>100 && reference.centers[0].size()==reference.centers[1].size(),"compares equally supported common section");
    auto swapped=measure(second,first);
    check(swapped.success && std::abs(swapped.angleDegrees-reference.angleDegrees)<1e-7,"seam exchange preserves unsigned angle");
    auto reversed=cylinder();reversed.axis=-reversed.axis;
    auto reverseResult=measure(first,second,reversed);
    check(reverseResult.success && std::abs(reverseResult.angleDegrees-reference.angleDegrees)<1e-6,"cylinder axis reversal preserves unsigned angle");
    auto radial=measure(seam(0,0,4),seam(2,25,-3));
    check(radial.success && std::abs(radial.angleDegrees-2)<.004,"radial weld height does not become angular error");
    auto crossing=measure(seam(0,0,0,0,std::acos(-1.)-.01),seam(2,25,0,0,std::acos(-1.)-.01));
    check(crossing.success && std::abs(crossing.angleDegrees-2)<.004,"common cut keeps seams continuous across pi");
    auto shorter=measure(first,seam(2,25,0,0,0,100));
    check(shorter.success && shorter.comparisonLength<reference.comparisonLength && std::abs(shorter.angleDegrees-2)<.01,"unequal seam lengths use common coverage");
    auto dense=pcl::make_shared<Measure::Cloud>(*first);
    for (int repeat=0;repeat<5;++repeat) dense->insert(dense->end(),first->begin(),first->begin()+first->size()/2);
    auto density=measure(dense,second);
    check(density.success && std::abs(density.angleDegrees-reference.angleDegrees)<.02,"sampling density does not dominate fitted direction");
    auto outliers=pcl::make_shared<Measure::Cloud>(*second);CylinderSurfaceFrame f;f.init({0,0,0},{0,0,1},1940);
    for (int i=0;i<20;++i) { const Eigen::Vector3d p=f.unproject(100+i,(-75+i*8),0);outliers->emplace_back(p.x(),p.y(),p.z()); }
    auto robust=measure(first,outliers);
    std::cout<<"[OUTLIERS] angle="<<robust.angleDegrees<<" success="<<robust.success<<" "<<robust.message<<std::endl;
    check(robust.success && std::abs(robust.angleDegrees-2)<.03,"sparse outliers do not determine seam direction");
    auto shuffled=pcl::make_shared<Measure::Cloud>(*second);std::mt19937 random(71);std::shuffle(shuffled->begin(),shuffled->end(),random);
    auto ordering=measure(first,shuffled);
    check(ordering.success && std::abs(ordering.angleDegrees-reference.angleDegrees)<1e-6,"input point order does not affect measurement");
    auto rotatedFirst=pcl::make_shared<Measure::Cloud>(*first),rotatedSecond=pcl::make_shared<Measure::Cloud>(*second);
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();const Eigen::Vector3d translation(10000,-2000,500);
    for (auto cloud:{rotatedFirst,rotatedSecond}) for (auto& p:*cloud) { const Eigen::Vector3d q=rotation*Eigen::Vector3d(p.x,p.y,p.z)+translation;p.x=q.x();p.y=q.y();p.z=q.z(); }
    auto pose=cylinder();pose.point=translation;pose.axis=rotation*pose.axis;
    auto transformed=measure(rotatedFirst,rotatedSecond,pose);
    check(transformed.success && std::abs(transformed.angleDegrees-reference.angleDegrees)<.006,"rigid transform and large coordinates preserve angle");
    auto shifted=cylinder();shifted.point+=123*shifted.axis;
    auto origin=measure(first,second,shifted);
    check(origin.success && std::abs(origin.angleDegrees-reference.angleDegrees)<1e-7,"axis point position along axis does not alter angle");
    bool curves=true;
    for (int side=0;side<2;++side) for (const auto& p:reference.curves[side]) {
        double a,s,e;reference.frame.project(p,a,s,e);
        const Eigen::Vector2d delta=Eigen::Vector2d(s,a)-reference.linePoint[side];const auto u=reference.lineDirection[side];
        curves &= std::abs(delta.x()*u.y()-delta.y()*u.x())<1e-8 && std::abs(e-reference.radialOffset[side])<1e-8;
    }
    check(curves,"reprojected display curves are fitted directions on the reference cylinder");
    for (int phase:{0,5,20,40,60,75,100}) {
        Measure calc(first,second,cylinder());calc.setProgressCallback([phase](int current,int,const std::string&) { return current<phase; });const auto result=calc.compute();
        check(result.cancelled && !result.success && result.curves[0].empty(),"cancel at phase "+std::to_string(phase)+" leaves no valid result");
    }
    auto invalid=seam(0,0);(*invalid)[0].x=std::numeric_limits<float>::quiet_NaN();
    check(!measure(invalid,second).success,"nonfinite coordinate rejected");
    auto c=cylinder();c.radius=0;check(!measure(first,second,c).success,"invalid cylinder radius rejected");c=cylinder();c.axis.setZero();check(!measure(first,second,c).success,"invalid cylinder axis rejected");
    check(!measure(pcl::make_shared<Measure::Cloud>(),second).success,"empty seam rejected");
    check(!measure(first,seam(2,25,0,500)).success,"disjoint coverage rejected");
    check(!measure(seam(0,0,0,0,0,8),seam(2,25,0,0,0,8)).success,"too short common section rejected");
    auto blob=pcl::make_shared<Measure::Cloud>();
    for (int x=-20;x<=20;++x) for (int y=-20;y<=20;++y) { const auto p=f.unproject(y,x);blob->emplace_back(p.x(),p.y(),p.z()); }
    check(!measure(blob,second).success,"round point patch does not produce a false seam angle");
    auto bent=pcl::make_shared<Measure::Cloud>();
    for (int i=-60;i<=60;++i) for (int j=-3;j<=3;++j) { const auto p=f.unproject(.008*i*i+j*.7,i);bent->emplace_back(p.x(),p.y(),p.z()); }
    check(!measure(first,bent).success,"strongly bent seam cannot masquerade as a single direction");
    auto wrong=cylinder();wrong.radius=1500;
    check(!measure(first,second,wrong).success,"unrelated reference cylinder rejected");
}
static Measure::Cylinder readReference()
{
    std::ifstream file("analysis/weld_parallelism/reference_real.txt");std::string key;Measure::Cylinder result;
    while (file>>key) {
        if (key=="final") { file>>result.point.x()>>result.point.y()>>result.point.z()>>result.axis.x()>>result.axis.y()>>result.axis.z()>>result.radius;break; }
        std::string rest;std::getline(file,rest);
    }
    return result;
}
static const std::string root="/media/jamesyasr/Shared/大创/演示案例/3_焊缝平行度/";
static std::array<Measure::Cloud::Ptr,2> realSeams()
{
    std::array<Measure::Cloud::Ptr,2> clouds{pcl::make_shared<Measure::Cloud>(),pcl::make_shared<Measure::Cloud>()};
    for (int side=0;side<2;++side) if (pcl::io::loadPCDFile(root+"焊缝"+std::to_string(side+1)+".pcd",*clouds[side])<0) throw std::runtime_error("Cannot read real seam");
    return clouds;
}
static void realTests()
{
    auto clouds=realSeams();auto ref=readReference();check(ref.radius==1940,"real reference uses confirmed design radius");
    auto start=std::chrono::steady_clock::now();auto result=measure(clouds[0],clouds[1],ref);
    check(result.success,"real seams measurable using raw-cloud three-stage reference");
    if (!result.success) { std::cout<<result.message<<std::endl;return; }
    std::ofstream out("analysis/weld_parallelism/result_real.txt");out<<std::setprecision(17)<<"angle_degrees "<<result.angleDegrees<<"\ncomparison_length "<<result.comparisonLength<<"\nphi0 "<<result.frame.phi0()<<'\n';
    for (int side=0;side<2;++side) out<<"line "<<side<<' '<<result.linePoint[side].transpose()<<' '<<result.lineDirection[side].transpose()<<' '<<result.centerlineRms[side]<<' '<<result.centers[side].size()<<'\n';
    std::ofstream points("analysis/weld_parallelism/projected_real.csv"),centers("analysis/weld_parallelism/centers_real.csv");points<<"seam,s,a\n"<<std::setprecision(17);centers<<"seam,s,a\n"<<std::setprecision(17);
    for (int side=0;side<2;++side) {
        for (const auto& p:result.projected[side]) points<<side<<','<<p.x()<<','<<p.y()<<'\n';
        for (const auto& p:result.centers[side]) centers<<side<<','<<p.x()<<','<<p.y()<<'\n';
    }
    std::cout<<std::setprecision(12)<<"[REAL] angle="<<result.angleDegrees<<" length="<<result.comparisonLength<<" center_rms="<<result.centerlineRms[0]<<','<<result.centerlineRms[1]<<" ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<std::endl;
}
static void pump(int ms=90) { QElapsedTimer timer;timer.start();do { QApplication::processEvents();QThread::msleep(2); } while (timer.elapsed()<ms); }
static void click(QWidget* widget)
{
    const auto pos=qobject_cast<QCheckBox*>(widget) ? QPoint(8,widget->height()/2):widget->rect().center();
    QMouseEvent down(QEvent::MouseButtonPress,QPointF(pos),QPointF(widget->mapToGlobal(pos)),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier),up(QEvent::MouseButtonRelease,QPointF(pos),QPointF(widget->mapToGlobal(pos)),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(widget,&down);QApplication::sendEvent(widget,&up);pump();
}
static int guiTests(int argc,char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
    CloudForgeAnalyzer main;main.resize(1600,1000);main.show();pump();main.ClearAllPointCloud();
    auto clouds=realSeams();auto ref=readReference();
    auto background=pcl::make_shared<Measure::Cloud>();pcl::io::loadPCDFile(root+"圆柱壁面.pcd",*background);
    main.AddPointCloud("圆柱壁面",background,ColorManager(200,210,215));
    main.AddPointCloud("焊缝1",clouds[0],ColorManager(230,230,235));main.AddPointCloud("焊缝2",clouds[1],ColorManager(235,235,235));
    auto coeff=pcl::make_shared<pcl::ModelCoefficients>();coeff->values={float(ref.point.x()),float(ref.point.y()),float(ref.point.z()),float(ref.axis.x()),float(ref.axis.y()),float(ref.axis.z()),float(ref.radius)};
    main.cylinderResultsMap["real_three_stage"]=coeff;main.m_lastThreeStageCylinder="real_three_stage";
    auto view=main.ui->winOfAnalyzer;auto renderer=view->renderWindow()->GetRenderers()->GetFirstRenderer();auto camera=renderer->GetActiveCamera();
    const Eigen::Vector3d center=(Eigen::Vector3d(13,147,-445)+Eigen::Vector3d(-13,125,-442))*.5;
    CylinderSurfaceFrame frame;frame.init(ref.point,ref.axis,ref.radius);
    const Eigen::Vector3d delta=center-ref.point,radial=(delta-delta.dot(frame.axisDirection())*frame.axisDirection()).normalized();
    const Eigen::Vector3d position=center+radial*400;camera->SetPosition(position.data());camera->SetFocalPoint(center.data());camera->SetViewUp(frame.axisDirection().data());camera->ParallelProjectionOn();camera->SetParallelScale(95);renderer->ResetCameraClippingRange();view->renderWindow()->Render();
    auto tool=main.m_weldParallelTool.get();auto dock=main.findChild<QDockWidget*>("weldParallelDock");
    auto original1=main.viewer->getCloudActorMap()->at("焊缝1").actor,original2=main.viewer->getCloudActorMap()->at("焊缝2").actor;
    auto style=view->interactor()->GetInteractorStyle();auto picker=view->interactor()->GetPicker();double oldPosition[3];camera->GetPosition(oldPosition);
    auto open=[&]() { main.ui->measure_weld_parallel->trigger();pump(); };
    auto wait=[&]() { QElapsedTimer timer;timer.start();while (main.IsTaskRunning() && timer.elapsed()<15000) pump(10);pump();check(!main.IsTaskRunning(),"measurement finishes without blocking GUI"); };
    auto run=[&]() { click(dock->findChild<QPushButton*>("weldParallelMeasure"));wait(); };
    open();check(tool->isActive() && dock->isVisible() && !dock->isFloating(),"sidebar opens native dock in main window");
    check(dock->findChild<QComboBox*>("weldParallelFirst")->currentText()=="焊缝1" && dock->findChild<QComboBox*>("weldParallelSecond")->currentText()=="焊缝2","two seam roles selected correctly");
    check(dock->findChild<QComboBox*>("weldParallelReference")->currentText().contains("1940.0 mm"),"shared three-stage cylinder and radius visible");
    check(std::equal(oldPosition,oldPosition+3,camera->GetPosition()),"opening controls preserves camera pose");
    auto secondBox=dock->findChild<QComboBox*>("weldParallelSecond");const int selectedSecond=secondBox->currentIndex();
    secondBox->setCurrentIndex(dock->findChild<QComboBox*>("weldParallelFirst")->currentIndex());pump();
    check(!dock->findChild<QPushButton*>("weldParallelMeasure")->isEnabled(),"same seam cannot be selected for both roles");secondBox->setCurrentIndex(selectedSecond);pump();
    run();check(tool->actors_.size()==4 && !original1->GetVisibility() && !original2->GetVisibility(),"temporary point and curve actors replace only selected displays");
    auto plot=dock->findChild<QWidget*>("weldParallelPlot");
    check(plot->property("hasData").toBool() && plot->property("xPixelsPerMm").toDouble()==plot->property("yPixelsPerMm").toDouble(),"unfolded chart uses equal millimetre scales");
    const double initialScale=plot->property("xPixelsPerMm").toDouble();const QPointF middle=plot->rect().center();
    QWheelEvent wheel(middle,QPointF(plot->mapToGlobal(middle.toPoint())),QPoint(),QPoint(0,120),Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
    QApplication::sendEvent(plot,&wheel);pump();
    check(plot->property("xPixelsPerMm").toDouble()>initialScale && plot->property("xPixelsPerMm")==plot->property("yPixelsPerMm"),"wheel zoom preserves equal metric scales");
    const auto beforePan=plot->grab().toImage();
    QMouseEvent panDown(QEvent::MouseButtonPress,middle,QPointF(plot->mapToGlobal(middle.toPoint())),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent panMove(QEvent::MouseMove,middle+QPointF(30,20),QPointF(plot->mapToGlobal((middle+QPointF(30,20)).toPoint())),Qt::NoButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent panUp(QEvent::MouseButtonRelease,middle+QPointF(30,20),QPointF(plot->mapToGlobal((middle+QPointF(30,20)).toPoint())),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(plot,&panDown);QApplication::sendEvent(plot,&panMove);QApplication::sendEvent(plot,&panUp);pump();
    check(beforePan!=plot->grab().toImage(),"dragging pans the visible unfolded chart");click(dock->findChild<QPushButton*>("weldParallelReset"));
    check(std::abs(plot->property("xPixelsPerMm").toDouble()-initialScale)<1e-10,"reset button restores chart extent");
    check(main.m_resultOverlay->sections_.size()==1 && main.ui->textEdit_Show_CFmes->toPlainText().contains("展开面夹角"),"shared corner annotation and report show result");
    check(main.CloudMap.at("焊缝1")==clouds[0] && main.CloudMap.at("焊缝2")==clouds[1] && view->interactor()->GetInteractorStyle()==style && view->interactor()->GetPicker()==picker && std::equal(oldPosition,oldPosition+3,camera->GetPosition()),"calculation preserves source clouds camera and interaction");
    bool unpickable=true;for (const auto& actor:tool->actors_) unpickable &= !actor->GetPickable();check(unpickable,"decorative results cannot intercept point picking");
    auto interactor=view->interactor();interactor->SetEventInformation(view->width()/2,view->height()/2);interactor->InvokeEvent(vtkCommand::LeftButtonPressEvent);
    interactor->SetEventInformation(view->width()/2+30,view->height()/2+15);interactor->InvokeEvent(vtkCommand::MouseMoveEvent);interactor->InvokeEvent(vtkCommand::LeftButtonReleaseEvent);pump();
    check(!std::equal(oldPosition,oldPosition+3,camera->GetPosition()),"main viewport rotation remains available with result displayed");
    camera->SetPosition(oldPosition);camera->SetFocalPoint(center.data());camera->SetViewUp(frame.axisDirection().data());renderer->ResetCameraClippingRange();view->renderWindow()->Render();pump();
    const auto suffix=qEnvironmentVariable("WELD_PARALLEL_CAPTURE_SUFFIX","");check(main.grab().save("analysis/weld_parallelism/main_result"+suffix+".png"),"main result screenshot saved");
    click(dock->findChild<QCheckBox*>("weldParallelPoints"));check(original1->GetVisibility() && !tool->actors_[0]->GetVisibility(),"point highlight toggle restores original appearance");click(dock->findChild<QCheckBox*>("weldParallelPoints"));
    click(dock->findChild<QCheckBox*>("weldParallelCurves"));check(!tool->actors_[1]->GetVisibility(),"fitted curve visibility controlled by button");click(dock->findChild<QCheckBox*>("weldParallelCurves"));
    run();check(tool->actors_.size()==4 && main.m_resultOverlay->sections_.size()==1,"remeasurement replaces owned display without accumulation");
    main.ui->action_ed_clean2DActor->trigger();pump();check(!plot->property("hasData").toBool() && main.m_resultOverlay->isHidden() && tool->actors_.size()==4,"2D cleanup clears Qt chart and labels while retaining geometry");
    run();dock->setFloating(true);pump();check(dock->isFloating(),"native dock can float");check(dock->grab().save("analysis/weld_parallelism/floating_controls"+suffix+".png"),"floating controls screenshot saved");
    QKeyEvent escape(QEvent::KeyPress,Qt::Key_Escape,Qt::NoModifier);QApplication::sendEvent(dock,&escape);pump();check(!tool->isActive() && tool->actors_.empty() && original1->GetVisibility() && original2->GetVisibility(),"floating Esc restores source visibility and removes all results");
    dock->setFloating(false);open();run();click(dock->findChild<QPushButton*>("weldParallelClear"));check(tool->isActive() && tool->actors_.empty() && !plot->property("hasData").toBool(),"clear keeps input panel and removes all owned results");
    main.StartWeldParallelismMeasurement();main.CancelCurrentTask();wait();check(tool->actors_.empty() && tool->isActive() && !tool->isBusy(),"cancel discards pending result");
    main.StartWeldParallelismMeasurement();tool->stop();wait();check(!tool->isActive() && tool->actors_.empty(),"close during calculation cannot recreate late results");
    open();run();main.ui->action_ed_cleangall->trigger();pump();check(!tool->isActive() && tool->actors_.empty() && main.m_resultOverlay->isHidden(),"clear all removes geometry and Qt results together");
    open();check(!tool->isActive(),"empty scene cannot start measurement");
    main.close();pump();std::cout<<"pass="<<passed<<" fail="<<failed<<std::endl;return failed ? 1:0;
}
int main(int argc,char** argv)
{
    if (argc>1 && std::string(argv[1])=="--gui") return guiTests(argc,argv);
    pureTests();if (argc==1 || std::string(argv[1])!="--synthetic") realTests();
    std::cout<<"pass="<<passed<<" fail="<<failed<<std::endl;return failed ? 1:0;
}
