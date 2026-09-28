#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include "Dialog/ParamDialogCurvSeg.h"

class CurvatureSegmentation {
public:
    // 新增：分割参数(用于后台计算: 参数已在界面线程收集完毕, 不再弹对话框)
    struct CurvParams {
        int kSearch = 20;                 // k近邻数
        float smoothThreshold = 1.0f;     // 平滑度(法线夹角)阈值
        float curvatureThreshold = 0.1f;  // 曲率阈值
        int minClusterSize = 300;         // 最小聚类点数
    };

    CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointXYZ input_point);
    // 新增：已知参数构造(不弹参数对话框, 不自动计算) —— 配合 compute() 在工作线程中执行
    CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointXYZ input_point,
        const CurvParams& params);

    // 新增：执行曲率区域生长分割计算(可被工作线程调用)
    void compute();

    bool isCancelled = false;    // 新增：是否被取消(原有构造函数的取消/无效参数分支会置位)
    std::string message;
	pcl::PointCloud<pcl::PointXYZ>::Ptr getOutputCloud() const { return output_cloud; }

private:
    int  k_search;
    float smooth_threshold;
    float curvature_threshold;
    int min_cluster_size;
	ParamDialogCurvSeg* dialog; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;
	pcl::PointXYZ picked_point;

    void extractPlane();

};