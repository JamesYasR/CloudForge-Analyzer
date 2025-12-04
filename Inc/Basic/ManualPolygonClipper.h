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
#include <boost/signals2.hpp> // 新增：保存回调连接

class ManualPolygonClipper : public QObject {
    Q_OBJECT
public:
    ManualPolygonClipper(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        vtkSmartPointer<vtkRenderWindow> mainRenderWindow,
        vtkRenderWindowInteractor* mainInteractor, // 改为原生指针，不接管所有权
        pcl::visualization::PCLVisualizer* mainviewer,
        QObject* parent = nullptr
    );
    ~ManualPolygonClipper();

    void startClipping();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getClippedCloud() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRemainCloud() const;
    void cleanup();
signals:
    void clippingFinished();

private:

    // 不用 smart pointer 来持有 widget 的 interactor（QVTK 管理其生命周期）
    vtkRenderWindowInteractor* mainInteractor;
    vtkSmartPointer<vtkInteractorStyle> originalStyle;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped, cloud_remain;
    pcl::PointXYZ curP, lastP;
    bool flag;
    bool isPickingMode;
    unsigned int line_id;
    pcl::visualization::PCLVisualizer* viewer;

    // 保存回调连接，以便正确断开（避免传 nullptr 导致崩溃）
    boost::signals2::connection keyboard_connection;
    boost::signals2::connection mouse_connection;

    void getScreentPos(double* displayPos, double* world);
    int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    void projectInliers();
    void resetPolygon();

    //  键盘/鼠标 事件处理 
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event);

    //  静态转发  
    static void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    static void mouseCallback(const pcl::visualization::MouseEvent& event, void* cookie);
};