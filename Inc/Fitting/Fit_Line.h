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
    // 新增：拟合参数(用于后台计算: 参数已在界面线程收集完毕, 不再弹对话框)
    struct FitParams {
        float DistanceThreshold = 0.01f;  // RANSAC距离阈值
        int MaxIterations = 1000;         // RANSAC最大迭代次数
    };

    Fit_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC);
    // 新增：已知参数构造(不弹参数对话框, 不自动计算) —— 配合 compute() 在工作线程中执行
    Fit_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC, const FitParams& params);
    ~Fit_Line();

    // 新增：执行直线拟合计算(可被工作线程调用)
    void compute();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Outliers();
    Eigen::VectorXf Get_Coeff_in(); // 获取直线模型系数
    pcl::PointXYZ Get_StartPoint() const { return start_point; }
    pcl::PointXYZ Get_EndPoint() const { return end_point; }

    bool isCancelled = false;   // 新增：是否被取消(原有构造函数的取消/无效参数分支会置位)

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