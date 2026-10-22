#include "cases.h"
#include "BaselineFitPlane.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <iomanip>
#include "CloudForgeAnalyzer.h"
#include "ui_CloudForgeAnalyzer.h"
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QThread>
#include <QTimer>
#include <vtkCamera.h>

static int passed=0, failed=0;
static void check(bool ok, const std::string& text)
{
    ok ? ++passed : ++failed;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << text << std::endl;
}
static bool equalCloud(const pcl::PointCloud<pcl::PointXYZ>& a, const pcl::PointCloud<pcl::PointXYZ>& b)
{
    if (a.size()!=b.size()) return false;
    for (size_t i=0;i<a.size();++i)
        if (std::memcmp(&a[i].x,&b[i].x,3*sizeof(float))) return false;
    return true;
}
template<class T> static void run(T& fit) { fit.compute(); }
static void compare(int number, int side)
{
    auto item=planeCase(number,side);
    BaselineFitPlane::FitParams p;
    p.LocalRadius=0; p.AnomalyThreshold=item.oldAnomalyThreshold;
    p.PlaneFitThreshold=item.params.PlaneFitThreshold; p.MaxIterations=item.params.MaxIterations;
    BaselineFitPlane old(item.cloud,p);
    auto start=std::chrono::steady_clock::now(); old.compute();
    const double oldMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    Fit_Plane current(item.cloud,item.params);
    start=std::chrono::steady_clock::now(); current.compute();
    const double newMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    const auto a=old.Get_Coeff_in(), b=current.Get_Coeff_in();
    const bool coefficients=a.size()==b.size() && !std::memcmp(a.data(),b.data(),a.size()*sizeof(float));
    check(coefficients && equalCloud(*old.Get_Inliers(),*current.Get_Inliers())
          && equalCloud(*old.Get_Outliers(),*current.Get_Outliers())
          && old.ComputeRMSE()==current.ComputeRMSE(), item.name+" coefficients, ordered clouds and RMSE bit-for-bit");
    std::cout << "[TIME] case=" << number << " N=" << item.cloud->size() << " old_ms=" << oldMs << " new_ms=" << newMs
        << " old_outliers=" << old.Get_Outliers()->size() << " new_outliers=" << current.Get_Outliers()->size() << std::endl;
}
static void pureChecks()
{
    auto item=planeCase(2);
    for (int cutoff : {0,1,10,85,94,100}) {
        Fit_Plane fit(item.cloud,item.params);
        fit.setProgressCallback([cutoff](int current,int,const std::string&) { return current<cutoff; });
        fit.compute();
        check(fit.isCancelled && fit.Get_Coeff_in().size()==0 && fit.Get_Inliers()->empty()
              && fit.Get_Outliers()->empty(),"cancel at phase "+std::to_string(cutoff)+" discards every partial result");
    }
    Fit_Plane repeat(item.cloud,item.params);
    int previous=-1; bool monotonic=true;
    repeat.setProgressCallback([&](int current,int total,const std::string&) { monotonic &= current>=previous && current<=total; previous=current; return true; });
    repeat.compute();
    auto first=*repeat.Get_Inliers(); auto c=repeat.Get_Coeff_in();
    check(monotonic && previous==100,"progress advances monotonically to completion");
    repeat.setProgressCallback({}); repeat.compute();
    check(equalCloud(first,*repeat.Get_Inliers()) && (c==repeat.Get_Coeff_in()),"repeat compute resets state without accumulating old results");
    repeat.setProgressCallback([](int,int,const std::string&) { return false; });repeat.compute();
    repeat.setProgressCallback({});repeat.compute();
    check(!repeat.isCancelled && equalCloud(first,*repeat.Get_Inliers()),"cancelled fitter can be reused");
    auto empty=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int mode=0;mode<7;++mode) {
        auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*item.cloud); auto p=item.params;
        if (mode==0) cloud=empty;
        if (mode==1) cloud->resize(2);
        if (mode==2) (*cloud)[0].x=std::numeric_limits<float>::quiet_NaN();
        if (mode==3) p.MaxIterations=0;
        if (mode==4) p.PlaneFitThreshold=0;
        if (mode==5) p.PlaneFitThreshold=std::numeric_limits<float>::infinity();
        if (mode==6) { cloud->clear(); for (int i=0;i<20;++i) cloud->emplace_back(i,0,0); }
        Fit_Plane invalid(cloud,p);invalid.compute();
        check(invalid.Get_Coeff_in().size()==0 && invalid.Get_Inliers()->empty() && invalid.message.find("不可拟合")!=std::string::npos,"invalid input "+std::to_string(mode)+" has one result-level failure and no model");
    }
}
static void pump(int ms=80)
{
    QElapsedTimer time;time.start();
    do { QApplication::processEvents();QThread::msleep(2); } while (time.elapsed()<ms);
}
static void click(QWidget* widget)
{
    const auto pos=widget->rect().center();
    QMouseEvent down(QEvent::MouseButtonPress,QPointF(pos),QPointF(widget->mapToGlobal(pos)),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent up(QEvent::MouseButtonRelease,QPointF(pos),QPointF(widget->mapToGlobal(pos)),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(widget,&down);QApplication::sendEvent(widget,&up);
}
static int guiChecks(int argc,char** argv)
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
    CloudForgeAnalyzer main;main.resize(1450,1000);main.show();pump();
    auto item=planeCase(2,64);
    main.ClearAllPointCloud();main.AddPointCloud("plane_sample",item.cloud,ColorManager(220,230,235));
    auto renderer=main.ui->winOfAnalyzer->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto camera=renderer->GetActiveCamera();camera->SetPosition(16,-45,50);camera->SetFocalPoint(16,16,0);camera->SetViewUp(0,0,1);
    renderer->ResetCameraClippingRange();main.ui->winOfAnalyzer->renderWindow()->Render();pump();
    auto input=main.CloudMap.at("plane_sample");
    Fit_Plane expected(item.cloud,item.params);expected.compute();
    int choices=0, parameters=0; bool difficultRun=false;
    QTimer dialogs;
    QObject::connect(&dialogs,&QTimer::timeout,[&]() {
        auto dialog=qobject_cast<QDialog*>(QApplication::activeModalWidget());if (!dialog) return;
        if (auto choose=qobject_cast<ChoseCloudDialog*>(dialog)) {
            choose->clearAllCheckBoxes();
            for (auto box:choose->findChildren<QCheckBox*>()) if (box->text()=="plane_sample") box->setChecked(true);
            ++choices;
        } else if (auto parametersDialog=qobject_cast<ParamDialog_FittingPlane*>(dialog)) {
            ++parameters;
            check(parametersDialog->paramEdits.size()==2,"plane parameters expose only distance threshold and iteration count");
            if (difficultRun) { parametersDialog->paramEdits[0]->setText("0.001");parametersDialog->paramEdits[1]->setText("100000"); }
        }
        else return;
        for (auto button:dialog->findChildren<QPushButton*>()) if (button->text()=="确定") { click(button);break; }
    });
    auto begin=[&]() { dialogs.start(20);main.ui->action_fit_plane->trigger();dialogs.stop(); };
    auto wait=[&]() { QElapsedTimer time;time.start();while (main.IsTaskRunning() && time.elapsed()<20000) pump(10);pump();check(!main.IsTaskRunning(),"desktop plane task returns without blocking interaction"); };
    begin();wait();
    check(choices==1 && parameters==1,"existing cloud and parameter dialogs work through visible confirm buttons");
    check(main.CloudMap.count("plane_fit_inliers") && equalCloud(*main.CloudMap.at("plane_fit_inliers"),*expected.Get_Inliers())
          && main.CloudMap.count("plane_fit_outliers") && equalCloud(*main.CloudMap.at("plane_fit_outliers"),*expected.Get_Outliers())
          && !main.CloudMap.count("plane_fit_anomaly"),"desktop displays exact green inliers and red outliers without obsolete anomaly cloud");
    check(main.CloudMap.at("plane_sample")==input && main.planeResultsMap.size()==1,"source preserved and successful plane registered once");
    check(main.ui->textEdit_Show_CFmes->toPlainText().contains("RMS误差") && main.ui->progressBar->value()==100,"result report and completed progress shown in existing UI");
    const auto suffix=qEnvironmentVariable("PLANE_CAPTURE_SUFFIX","");
    check(main.grab().save("analysis/plane_fitting/main_result"+suffix+".png"),"desktop result screenshot saved");
    main.DelePointCloud("plane_sample");
    auto flat=planeCase(0);
    main.AddPointCloud("plane_sample",flat.cloud,ColorManager(220,230,235));
    begin();wait();
    check(main.CloudMap.at("plane_fit_inliers")->size()==flat.cloud->size() && !main.CloudMap.count("plane_fit_outliers"),
          "repeated fitting replaces inliers and removes previous red outliers when none remain");
    difficultRun=true;
    main.ClearAllPointCloud();
    auto difficult=planeCase(14,150).cloud;
    main.AddPointCloud("plane_sample",difficult,ColorManager(220,230,235));
    const auto planeCount=main.planeResultsMap.size();
    begin();
    QElapsedTimer progress;progress.start();
    while (main.IsTaskRunning() && main.m_asyncState->current.load()<8 && progress.elapsed()<5000) pump(10);
    check(main.IsTaskRunning() && main.m_asyncState->current.load()>=5,"global fit publishes live progress");
    main.ui->action_cancel_task->trigger();
    QElapsedTimer cancel;cancel.start();
    while (!main.m_asyncFuture.isFinished() && cancel.elapsed()<1500) QThread::msleep(1);
    const auto stopMs=cancel.elapsed();const bool stopped=main.m_asyncFuture.isFinished();
    wait();
    std::cout<<"[CANCEL] worker_stop_ms="<<stopMs<<" clouds="<<main.CloudMap.size()<<" planes="<<main.planeResultsMap.size()<<" before="<<planeCount<<std::endl;
    check(stopped && stopMs<1500 && main.CloudMap.size()==1 && main.planeResultsMap.size()==planeCount,"toolbar cancel promptly stops calculation and adds no partial clouds or plane");
    check(!main.ui->action_cancel_task->isEnabled(),"cancel action resets after task completion");
    main.close();pump();
    std::cout << "pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1:0;
}
int main(int argc,char** argv)
{
    if (argc>1 && std::string(argv[1])=="--gui") return guiChecks(argc,argv);
    if (argc>1) compare(std::stoi(argv[1]),argc>2 ? std::stoi(argv[2]) : 100);
    else { for (int i=0;i<18;++i) compare(i,32);pureChecks(); }
    std::cout << "pass=" << passed << " fail=" << failed << std::endl;
    return failed ? 1 : 0;
}
