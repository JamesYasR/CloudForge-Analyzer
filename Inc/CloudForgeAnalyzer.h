#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_CloudForgeAnalyzer.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkOutputWindow.h>
#include "QDir"
#include "QFileDialog"
#include <string>
#include <filesystem>
#include "Protrusion_Depression_Cylinder.h"
#include "Linear_Depression_Plane.h"
#include "funcs.h"
#include "PreProcessing/PreProcessing.h"
#include "Fitting/Fit_Cylinder.h"
#include "ColorManager.h"

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
    void Slot_CurveFittingPC();
    void Slot_ChangeVA_x();
    void Slot_ChangeVA_y();
    void Slot_ChangeVA_z();
    void Slot_ChangeVA_o();
    void Slot_fi_open_Triggered();
    void Slot_fi_save_Triggered();
    void Slot_fi_saveas_Triggered();
    void Slot_ph_1_Triggered();
    void Slot_fl_1_Triggered();
    void Slot_fl_2_Triggered();
    void Slot_ed_dork_Triggered();
    void Slot_fit_cy_Triggered();

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

    //          
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr renderer_custom;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;
    std::map<std::string, ColorManager> ColorMap;
    

};
