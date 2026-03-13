#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <vtkSmartPointer.h>
#include <vtkCardinalSpline.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "Dialog/Dialog.h"

class MeasureArc
{
public:
    // === 拟合方法枚举 ===
    enum FitMethod {
        CARDINAL_SPLINE = 0,      // 现有方法：Cardinal样条插值
        BSPLINE_LSQ = 1           // 新方法：最小二乘B样条逼近
    };

    // 构造函数 - 新增拟合方法参数
    MeasureArc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::ModelCoefficients::Ptr cylinder_coeff,
        pcl::PointXYZ* specified_point = nullptr,
        FitMethod method = CARDINAL_SPLINE);

    // 执行计算
    void compute();

    // 获取可视化Actor
    vtkSmartPointer<vtkActor> getVisualizationActor() const { return m_splineActor; }

    // 公共成员变量 (结果与状态)
    double arcLength = 0.0;
    bool success = false;
    bool isCancelled = false;
    std::string message;

    // === 可调参数 ===
    // 通用参数
    double sliceThicknessFactor = 5.0;    // 切片厚度因子
    double integrationTolerance = 1e-5;   // 积分容差
    double splineLineWidth = 3.0;         // 可视化线宽
    double splineColor[3] = { 1.0, 0.0, 0.0 }; // 曲线颜色 (红)

    // Cardinal样条参数
    int downsampleTargetSize = 50;           // 降采样目标点数
    double virtualPointExtrapolation = 0.05; // 虚拟点外推比例

    // === B样条逼近参数 ===
    FitMethod fitMethod = CARDINAL_SPLINE;  // 拟合方法选择
    int bsplineDegree = 3;                   // B样条次数 (默认3次)
    int bsplineControlPoints = 15;          // B样条控制点数量
    double bsplineSmoothingFactor = 0.01;   // 平滑因子 (λ), 0=插值, >0=平滑

private:
    // 核心计算步骤
    bool parseCylinderParameters();
    bool determineMeasurementPlane(pcl::PointXYZ* specified_point);
    bool extractAndProjectSlice();
    bool sortProjectedPoints();

    // 样条拟合方法
    bool fitSplineCurve();                     // 主入口，根据fitMethod选择
    bool fitCardinalSpline();                  // 现有Cardinal样条方法
    bool fitBSplineLeastSquares();            // 新增：最小二乘B样条逼近

    bool computeArcLengthByIntegration();
    void createVisualizationActor();

    // B样条辅助函数
    double bsplineBasis(int i, int k, double t, const std::vector<double>& knots);
    std::vector<double> generateUniformKnots(int n, int p);
    double evaluateBSpline(double t, const std::vector<double>& coeffsX,
        const std::vector<double>& coeffsY,
        const std::vector<double>& knots);
    std::pair<double, double> evaluateBSplineDerivative(double t,
        const std::vector<double>& coeffsX,
        const std::vector<double>& coeffsY,
        const std::vector<double>& knots);

    // 输入数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_inputCloud;
    pcl::ModelCoefficients::Ptr m_cylinderCoeff;
    pcl::PointXYZ* m_specifiedPoint;

    // 内部计算数据
    Eigen::Vector3f m_axisPoint;
    Eigen::Vector3f m_axisDirection;
    double m_cylinderRadius;
    Eigen::Vector3f m_planeCenter;
    Eigen::Vector3f m_planeX, m_planeY;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_sliceCloud;
    std::vector<Eigen::Vector2f> m_projectedPoints;
    std::vector<Eigen::Vector2f> m_sortedPoints;
    std::vector<Eigen::Vector2f> m_fittedPoints2D;

    // 样条数据存储
    vtkSmartPointer<vtkCardinalSpline> m_vtkSplineX;
    vtkSmartPointer<vtkCardinalSpline> m_vtkSplineY;

    // B样条数据存储
    std::vector<double> m_bsplineCoeffsX;  // X方向控制点系数
    std::vector<double> m_bsplineCoeffsY;  // Y方向控制点系数
    std::vector<double> m_bsplineKnots;    // 节点向量

    vtkSmartPointer<vtkActor> m_splineActor;

    // 参数范围
    double m_actualParamStart = 0.0;
    double m_actualParamEnd = 1.0;
    bool m_hasVirtualPoints = false;

    // B样条参数范围
    double m_bsplineParamStart = 0.0;
    double m_bsplineParamEnd = 1.0;
};