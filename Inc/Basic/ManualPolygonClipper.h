#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QObject>
#include <QString>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>  // 必须添加
#include <vtkInteractorStyleSwitch.h> // 建议添加

class ManualPolygonClipper : public QObject {
    Q_OBJECT
public:
    ManualPolygonClipper(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        vtkSmartPointer<vtkRenderWindow> mainRenderWindow,
        vtkSmartPointer<vtkRenderWindowInteractor> mainInteractor,
        pcl::visualization::PCLVisualizer* mainviewer,
        QObject* parent = nullptr
    );
    ~ManualPolygonClipper();

    void startClipping();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getClippedCloud() const;
    void cleanup();
signals:
    void clippingFinished();

private:

    vtkSmartPointer<vtkRenderWindowInteractor> mainInteractor;  // 新增
    vtkSmartPointer<vtkInteractorStyle> originalStyle;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped;
    pcl::PointXYZ curP, lastP;
    bool flag;
    bool isPickingMode;
    unsigned int line_id;
    pcl::visualization::PCLVisualizer* viewer;

    void getScreentPos(double* displayPos, double* world);
    int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    void projectInliers();
    void resetPolygon();

    //  ¼  ص 
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event);

    //   ̬ת  
    static void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    static void mouseCallback(const pcl::visualization::MouseEvent& event, void* cookie);
};