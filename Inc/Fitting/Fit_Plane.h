#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>  
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Dialog/Dialog.h"
#include <unordered_set>
#include <iostream>

class Fit_Plane {
public:
    Fit_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC);
    ~Fit_Plane();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Outliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_AnomalyPoints();  // 获取异常点
    Eigen::VectorXf Get_Coeff_in();
    float Get_Inliers_Percentage();
    float Get_Outliers_Percentage();
    float Get_Anomaly_Percentage();  // 获取异常点百分比
    float ComputeRMSE();
    float ComputePlanarityScore();  // 计算平面度评分

    bool isCancelled = false;
    std::string message;       // 存储所有信息
private:
    float LocalRadius;         // 局部拟合半径
    float AnomalyThreshold;    // 新增：异常点判定阈值
    float PlaneFitThreshold;   // 重命名：平面拟合距离阈值
    int MaxIterations;

    Eigen::VectorXf coeff_in;  // 平面系数 [A, B, C, D]

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_anomaly;  // 异常点

    ParamDialog_FittingPlane* dialog;

    void Proc();
    void detectAnomalyPoints();  // 异常点检测
    void fitPlaneWithRANSAC();   // RANSAC平面拟合
};