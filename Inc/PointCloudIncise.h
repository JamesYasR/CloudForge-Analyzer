#include <pcl/visualization/pcl_visualizer.h>
#include <QObject>
#include <vtkCamera.h>

class PointCloudClipper : public QObject {
    Q_OBJECT
public:
    explicit PointCloudClipper(QObject* parent = nullptr);
    pcl::PointXYZ projectPointToPlane(const pcl::PointXYZ& point);
    Eigen::Vector2f projectToLocal(const pcl::PointXYZ& projPoint);
    void computeLocalCoordinateSystem();
    void setupClipper(pcl::visualization::PCLVisualizer::Ptr viewer,
        pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud,
        vtkRenderWindow* renderWindow);

    void startClipping();
    void cancelClipping();
    void applyClipping();
    bool isClippingActive() const;
    bool polygonEmpty() const;
    void setClipPlaneHeight(double height) { clipPlaneHeight = height; }
signals:
    void clippingFinished(bool success);
    void updateButtonStates(bool isClipping);

private:
    Eigen::Vector3f planeNormal;  // 裁切平面法向量（摄像头视线方向）
    Eigen::Vector3f planeOrigin;  // 裁切平面原点（焦点位置）
    Eigen::Vector3f localU;       // 局部坐标系U轴
    Eigen::Vector3f localV;       // 局部坐标系V轴


    double clipPlaneHeight = 0.0; // 默认平面高度
    // 裁剪状态管理
    enum ClippingState { IDLE, DRAWING, CONFIRM };
    ClippingState currentState;

    // 可视化相关
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkRenderWindow* renderWindow;
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clippedCloud;

    // 多边形坐标
    std::vector<pcl::PointXYZ> polygonPoints;

    // 事件回调函数
    void keyboardEvent(const pcl::visualization::KeyboardEvent& event);
    void mouseEvent(const pcl::visualization::MouseEvent& event);
    void convertScreenToWorld(const int x, const int y, double* worldPos);
    bool pointInPolygon(const pcl::PointXYZ& point);

    // 辅助函数
    void updateVisualization();
    void clearTempVisuals();
};


