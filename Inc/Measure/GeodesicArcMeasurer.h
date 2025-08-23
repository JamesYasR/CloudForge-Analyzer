#ifndef GEODESIC_ARC_MEASURER_H
#define GEODESIC_ARC_MEASURER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <Eigen/Core>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkAssembly.h>  // 添加Assembly头文件
#include <vtkProp.h>      // 添加通用Prop基类
#include <vtkTextProperty.h>
#include <vtkTextActor.h>

// 前向声明VTK类
class vtkRenderer;
class vtkPolyData;

class GeodesicArcMeasurer
{
public:
    // 曲面点结构
    struct SurfacePoint {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        float curvature = 0.0f;
    };

    // 测量结果结构
    struct MeasurementResult {
        double arc_length = 0.0;
        std::vector<Eigen::Vector3f> path_points;
        bool success = false;
    };

    // 构造函数
    GeodesicArcMeasurer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        double search_radius = 0.03);

    // 测量两点间的弧长
    MeasurementResult measureArcLength(const pcl::PointXYZ& start, const pcl::PointXYZ& end);

    // 获取曲面模型
    const std::vector<SurfacePoint>& getSurfaceModel() const;

    // 创建可视化对象 - 使用通用vtkProp类型
    std::pair<std::vector<vtkSmartPointer<vtkProp>>,
        std::vector<vtkSmartPointer<vtkTextActor>>>
        createVisualizationActors(
            const MeasurementResult& result,
            bool show_surface = true,
            bool show_path = true,
            bool show_points = true,
            bool show_legend = true);
    void setRadiusScaleFactors(double curvature_scale, double geodesic_scale);
    void setCurvatureRadius(double radius);
    void setGeodesicRadius(double radius);
private:
    double base_radius_;          // 核心基准半径（构造函数传入）
    double curvature_radius_;     // 曲率计算半径
    double geodesic_radius_;      // 测地线搜索半径
    double curvature_scale;
    double geodesic_scale;
    pcl::KdTreeFLANN<pcl::PointXYZ> surface_kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    std::vector<SurfacePoint> surface_points_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    void buildSurfaceModel(double search_radius);
    void computeCurvature();
    int findNearestSurfacePoint(const pcl::PointXYZ& point);
    double computeGeodesicDistance(int start_idx, int end_idx, std::vector<Eigen::Vector3f>& path_points);

    // 可视化辅助函数 - 修正返回类型
    vtkSmartPointer<vtkActor> createSurfaceActor();
    vtkSmartPointer<vtkActor> createPointCloudActor();
    vtkSmartPointer<vtkActor> createPathActor(const std::vector<Eigen::Vector3f>& path_points);
    vtkSmartPointer<vtkAssembly> createStartEndActors(const Eigen::Vector3f& start, const Eigen::Vector3f& end);  // 改为返回Assembly
    vtkSmartPointer<vtkTextActor> createTextActor(double arc_length);
};

#endif // GEODESIC_ARC_MEASURER_H