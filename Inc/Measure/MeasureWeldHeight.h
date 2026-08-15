#pragma once
#include "config/pcl114.h"
#include <memory>
#include <vector>

class MeasureWeldHeight
{
public:
    // ============ 数据结构 ============

    struct GridCell {
        Eigen::Vector3i index;                          // 网格坐标 (ix, iy, iz)
        pcl::PointCloud<pcl::PointXYZ>::Ptr weld_points; // 本格内的焊缝点
        Eigen::Vector4f plane_coeffs;                    // 拟合的局部平面 [A,B,C,D] 归一化
        bool plane_valid;                                // 平面是否有效
        Eigen::Vector3f base_centroid;                   // 参与拟合的壁面点质心
        Eigen::Vector3f weld_centroid;                   // 本格内焊缝点质心
        GridCell() : weld_points(new pcl::PointCloud<pcl::PointXYZ>),
            plane_coeffs(Eigen::Vector4f::Zero()), plane_valid(false),
            base_centroid(Eigen::Vector3f::Zero()), weld_centroid(Eigen::Vector3f::Zero()) {}
    };

    struct AssessmentResult {
        double min_height;               // 最小高度（最深的凹陷，负值）
        double max_height;               // 最大高度（最高的凸起，正值）
        double mean_height;              // 平均高度（有符号）
        double rms_height;               // 均方根高度
        double std_height;               // 高度标准差
        pcl::PointXYZ highest_point;     // 最高点坐标
        double highest_value;            // 最高点高度值
        pcl::PointXYZ lowest_point;      // 最低点坐标（最深凹陷）
        double lowest_value;             // 最低点高度值
        int valid_points;                // 有效评估点数
        int total_points;                // 焊缝总点数
        std::string assessment_message;  // 评估报告文本

        AssessmentResult() : min_height(0.0), max_height(0.0), mean_height(0.0),
            rms_height(0.0), std_height(0.0),
            highest_point(pcl::PointXYZ()), highest_value(0.0),
            lowest_point(pcl::PointXYZ()), lowest_value(0.0),
            valid_points(0), total_points(0) {}
    };

    // ============ 构造/析构 ============
    MeasureWeldHeight();
    ~MeasureWeldHeight();

    // ============ 参数设置 ============
    void setWeldCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setBaseCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setSearchRadius(double radius_mm);
    void setRegionSize(double size_mm);
    void setRansacThreshold(double threshold_mm);
    void setMinNeighbors(int min_n);
    void setVerbose(bool verbose);

    // ============ 执行评估 ============
    AssessmentResult evaluate();

    // ============ 结果获取 ============
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getHeatMapCloud() const { return heatmap_cloud_; }
    void getHeightRange(double& min_h, double& max_h) const {
        min_h = min_height_; max_h = max_height_;
    }
    double getMaxAbsHeight() const { return max_abs_height_; }
    AssessmentResult getLastResult() const { return last_result_; }

private:
    // ============ 核心算法步骤 ============
    void buildGrid();                         // 步骤1: 空间网格划分
    void fitLocalPlanes();                    // 步骤2: 每格拟合局部平面
    void measureAllPoints();                  // 步骤3: 逐点计算高度
    void generateHeatMap();                   // 生成热力图点云

    // ============ 工具函数 ============
    Eigen::Vector4f fitPlaneRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void orientPlaneNormal(Eigen::Vector4f& plane, const Eigen::Vector3f& weld_centroid,
        const Eigen::Vector3f& base_centroid);
    double signedDistanceToPlane(const pcl::PointXYZ& point, const Eigen::Vector4f& plane);
    void getColorForSignedDistance(double dist, double max_abs,
        uint8_t& r, uint8_t& g, uint8_t& b) const;
    bool isPointValid(const pcl::PointXYZ& point) const;
    void printDebug(const std::string& msg) const;
    Eigen::Vector3f computeCloudCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // ============ 输入数据 ============
    pcl::PointCloud<pcl::PointXYZ>::Ptr weld_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr base_kdtree_;

    // ============ 参数 ============
    double search_radius_;        // 搜索壁面点半径 (mm)
    double region_size_;          // 网格边长 (mm)
    double ransac_threshold_;     // RANSAC 内点阈值 (mm)
    int min_neighbors_;           // 最少邻域点数
    bool verbose_;

    // ============ 中间结果 ============
    std::vector<GridCell> grid_cells_;
    Eigen::Vector3f grid_origin_;   // 网格原点
    Eigen::Vector3i grid_dims_;     // 网格维度
    Eigen::Vector3f cell_size_vec_; // 单格实际尺寸

    // ============ 输出结果 ============
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud_;
    double min_height_;
    double max_height_;
    double max_abs_height_;         // |height| 的最大值（用于颜色映射）
    AssessmentResult last_result_;
};
