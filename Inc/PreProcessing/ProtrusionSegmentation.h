#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include "Dialog/​​ParamDialogProtrusion.h"
#include <Eigen/Dense>

class ProtrusionSegmentation {
public:
    /**
     * @brief 基于曲面突起的焊缝分割
     * @param input_cloud 输入点云
     */
    ProtrusionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

    /**
     * @brief 检查分割器是否有效
     * @return 参数有效返回true，否则false
     */
    bool isValid() const { return is_valid; }

    /**
     * @brief 执行突起分割
     */
    void segment();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlanarCloud() const { return planar_cloud; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getProtrusionCloud() const { return protrusion_cloud; }

private:
    float search_radius;        // 局部拟合的搜索半径
    float height_threshold;     // 突出高度阈值
    int min_cluster_size;       // 最小聚类点数
    ParamDialogProtrusion* dialog; // 参数设置对话框
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud;    // 平面区域
    pcl::PointCloud<pcl::PointXYZ>::Ptr protrusion_cloud; // 焊缝/突起区域

    bool is_valid = false;      // 分割器状态标志
};