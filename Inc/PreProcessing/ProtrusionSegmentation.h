#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include "Dialog/ParamDialogProtrusion.h"
#include <Eigen/Dense>

class ProtrusionSegmentation {
public:
    // 新增：分割参数(用于后台计算: 参数已在界面线程收集完毕, 不再弹对话框)
    struct Params {
        float heightThreshold = 0.2f;   // 突出高度阈值(对话框默认值)
        float searchRadius = 16.0f;     // 局部拟合搜索半径(对话框默认值)
        int minClusterSize = 30;        // 最小聚类点数(对话框默认值)
    };

    /**
     * @brief 基于曲面突起的焊缝分割
     * @param input_cloud 输入点云
     */
    ProtrusionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

    // 新增：已知参数构造(不弹参数对话框, 不自动计算) —— 配合 segment()/compute() 在工作线程中执行
    ProtrusionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const Params& params);

    /**
     * @brief 检查分割器是否有效
     * @return 参数有效返回true，否则false
     */
    bool isValid() const { return is_valid; }

    /**
     * @brief 执行突起分割
     */
    void segment();

    // 新增：执行突起分割计算(等价于 segment(), 便于统一在工作线程中调用)
    void compute() { segment(); }

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