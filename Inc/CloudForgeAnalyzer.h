#pragma once
#include "config/qt6.h"
#include "config/pcl114.h"
#include "config/vtk9.h"
#include "Fitting/Fitting.h"
#include "PreProcessing/PreProcessing.h"
#include "Measure/Measure.h"
#include "Basic/Basic.h"


#include <string>
#include <filesystem>
#include "Protrusion_Depression_Cylinder.h"
#include "Linear_Depression_Plane.h"
#include "funcs.h"



QT_BEGIN_NAMESPACE
namespace Ui { class CloudForgeAnalyzerClass; };
QT_END_NAMESPACE

class CloudForgeAnalyzer : public QMainWindow
{
    Q_OBJECT

public:
    CloudForgeAnalyzer(QWidget *parent = nullptr);
    ~CloudForgeAnalyzer();
    Ui::CloudForgeAnalyzerClass* ui;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private slots:
    //void Slot_CurveFittingPC();
    void Slot_ChangeVA_x();
    void Slot_ChangeVA_y();
    void Slot_ChangeVA_z();
    void Slot_ChangeVA_o();
    void Slot_fi_open_Triggered();
    void Slot_fi_save_Triggered();
    void Slot_fi_saveas_Triggered();
    void Slot_fi_add_Triggered();
    void Slot_ph_1_Triggered();
    void Slot_fl_1_Triggered();
    void Slot_fl_2_Triggered();
    void Slot_ed_dork_Triggered();
    void Slot_ed_cleangeo_Triggered();
    void Slot_ed_cleanall_Triggered();
    void Slot_fit_cy_Triggered();
    void Slot_ph_CurvSeg_Triggered();

    void Tool_SetMeasureCylinder();
    void Tool_MeasureGeodisic();
    void Update_PointCounts();
private:
    //        
    void AddPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ColorManager color);
    void ClearAllPointCloud();
    void DelePointCloud(std::string name);
    void InitalizeQWidgets();  
    void InitalizeRenderer();
    void InitalizeConnects();
    void TeEDebug(std::string debugMes);
    void UpdateCamera(int a, int b, int c);
    void Update_CFmes(std::string cfmes);
    void mainLoop_Init();
    //          
    void InitializeProgressBar();
    void SetProgressBarValue(int percentage, const QString& message = "");
    void ResetProgressBar();
    

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr renderer_custom;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;
    std::map<std::string, ColorManager> ColorMap;

    bool isCylinderMeasure = false;
    

};
