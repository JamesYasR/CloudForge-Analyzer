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
    // 构造函数
    MeasureArc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::ModelCoefficients::Ptr cylinder_coeff,
        pcl::PointXYZ* specified_point = nullptr);

    // 执行计算
    void compute();

    // 获取可视化Actor
    vtkSmartPointer<vtkActor> getVisualizationActor() const { return m_splineActor; }

    // 公共成员变量 (结果与状态)
    double arcLength = 0.0;
    bool success = false;
	bool isCancelled = false; // 新增：操作取消标志
    std::string message;

    // --- 可调参数 ---
    double sliceThicknessFactor = 5.0;    // 切片厚度因子
    double integrationTolerance = 1e-5;   // 积分容差
    double splineLineWidth = 3.0;         // 可视化线宽
    double splineColor[3] = { 1.0, 0.0, 0.0 }; // 曲线颜色 (红)

    // --- 新增暴露的核心算法参数 ---
    int downsampleTargetSize = 50;           // 降采样目标点数，控制曲线平滑度
    double virtualPointExtrapolation = 0.05; // 虚拟点外推比例，控制端点稳定性与边界
    // --- 参数结束 ---

private:
    // 核心计算步骤
    bool parseCylinderParameters();
    bool determineMeasurementPlane(pcl::PointXYZ* specified_point);
    bool extractAndProjectSlice();
    bool sortProjectedPoints();
    bool fitSplineCurve();
    bool computeArcLengthByIntegration();
    void createVisualizationActor();

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

    vtkSmartPointer<vtkCardinalSpline> m_vtkSplineX;
    vtkSmartPointer<vtkCardinalSpline> m_vtkSplineY;
    vtkSmartPointer<vtkActor> m_splineActor;

    // 新增：样条曲线实际点云的参数范围
    double m_actualParamStart = 0.0;
    double m_actualParamEnd = 1.0;
    bool m_hasVirtualPoints = false;

	ParamDialogMeaArc *dialog; // 用于获取参数
};