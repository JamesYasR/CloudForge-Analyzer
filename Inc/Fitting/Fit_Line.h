#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h> // 包含copyPointCloud的头文件
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include "Dialog/ParamDialogBase.h"
#include <unordered_set>

class ParamDialog_FittingLine; // 前向声明

class Fit_Line {
public:
    Fit_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC);
    ~Fit_Line();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Outliers();
    Eigen::VectorXf Get_Coeff_in(); // 获取直线模型系数
    pcl::PointXYZ Get_StartPoint() const { return start_point; }
    pcl::PointXYZ Get_EndPoint() const { return end_point; }

private:
    pcl::PointXYZ start_point;
    pcl::PointXYZ end_point;
    int MaxIterations;          // RANSAC最大迭代次数
    float DistanceThreshold;    // 距离阈值
    Eigen::VectorXf coeff_in;   // 模型系数
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;
    
    ParamDialog_FittingLine* paramDialog; // 参数对话框
    void Proc(); // 处理函数
};