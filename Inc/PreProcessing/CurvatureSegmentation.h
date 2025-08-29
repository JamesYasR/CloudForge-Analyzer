#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include "Dialog/ParamDialogCurvSeg.h"

class CurvatureSegmentation {
public:
    /**
     * @brief 构造函数
     * @param curvature_threshold 曲率阈值，区分平面/焊缝的关键参数
     * @param normal_radius 法线估计的搜索半径
     * @param min_cluster_size 最小聚类点数（过滤噪声点）
     */
    CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

    /**
     * @brief 执行曲率分割
     * @param input_cloud 输入点云
     * @param planar_cloud 输出平面区域点云
     * @param weld_cloud 输出焊缝痕迹点云
     */
    void segment();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPlanarCloud() const { return planar_cloud; }
	pcl::PointCloud<pcl::PointXYZ>::Ptr getNonPlanarCloud() const { return weld_cloud; }
private:
    float curvature_threshold;  // 曲率分割阈值
    float normal_radius;        // 法线估计半径
    int min_cluster_size;       // 最小聚类点数
	ParamDialogCurvSeg* dialog; // 参数设置对话框
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr weld_cloud;
};