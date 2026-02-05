#pragma once
#include "config/pcl114.h"
#include <memory>

class MeasureCylindricity
{
public:
    // 空间直线参数结构（4自由度）
    struct LineParams {
        Eigen::Vector3f point;      // 直线上一点（3个参数，但实际只有2个自由度）
        Eigen::Vector3f direction; // 方向向量（单位向量，2个自由度）
        // 总自由度 = 2（方向）+ 2（点在线上的位置）= 4

        LineParams() : point(Eigen::Vector3f::Zero()),
            direction(Eigen::Vector3f::UnitZ()) {
        }

        // 从系数数组构造
        LineParams(const Eigen::Vector3f& center, const Eigen::Vector3f& axis)
            : point(center), direction(axis.normalized()) {
        }

        // 获取系数数组 [point.x, point.y, point.z, direction.x, direction.y, direction.z]
        std::vector<float> toCoeffs() const {
            return { point.x(), point.y(), point.z(), direction.x(), direction.y(), direction.z() };
        }
    };

    struct AssessmentResult {
        double outlier_ratio;           // 不贴合点比例 (0.0-1.0)
        double min_deviation;
        double max_deviation;           // 最大偏差距离
        double mean_deviation;          // 平均偏差距离
        double rms_error;               // 均方根误差
        double uniformity_rms;         // 新增：均匀性RMSE（标准差）
        LineParams best_fit_line;       // 最优拟合直线参数
        bool is_acceptable;             // 是否在容差范围内
        std::string assessment_message; // 评估结果消息

        // 圆柱参数（由直线和设计半径推导）
        Eigen::Vector3f getCylinderAxisPoint() const { return best_fit_line.point; }
        Eigen::Vector3f getCylinderAxisDirection() const { return best_fit_line.direction; }
        double getDesignRadius() const { return design_radius; }

        // 获取轴线系数
        std::vector<float> getAxisCoeffs() const { return best_fit_line.toCoeffs(); }

        AssessmentResult() : outlier_ratio(0.0), min_deviation(0.0),max_deviation(0.0),
            mean_deviation(0.0), rms_error(0.0), uniformity_rms(0.0),
            design_radius(0.0), is_acceptable(false) {
        }

    private:
        double design_radius; // 设计半径（用于结果展示）
        friend class MeasureCylindricity;
    };

    // 获取带热力图颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getHeatMapCloud() const { return heatmap_cloud_; }

    // 获取距离范围用于颜色映射
    void getDistanceRange(double& min_distance, double& max_distance) const {
        min_distance = min_distance_;
        max_distance = max_distance_;
    }

    MeasureCylindricity();
    ~MeasureCylindricity();

    // 设置输入参数
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setDesignRadius(double radius);
    void setTolerance(double tolerance);
    void setMaxIterations(int max_iters);
    void setVerbose(bool verbose);

    // 新增：设置初始直线参数接口
    void setInitialLine(const Eigen::Vector3f& center, const Eigen::Vector3f& axis);
    void setInitialLine(const LineParams& line);
    void setInitialLineFromCoeffs(const std::vector<float>& coeffs);

    // 执行圆柱度评估（基于新思路）
    AssessmentResult evaluateCylindricity();

    // 新增：直接评估给定直线（不进行优化）
    AssessmentResult evaluateGivenLine(const Eigen::Vector3f& center, const Eigen::Vector3f& axis);

    // 获取中间结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr getInliers() const { return inliers_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getOutliers() const { return outliers_; }
    std::vector<double> getDistanceMap() const { return distance_map_; }

    // 新增：获取内点云和外点云（拼写修正）
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_inliers() const { return inliers_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_outliers() const { return outliers_; }

    // 新增：获取最终优化的轴线系数
    std::vector<float> getOptimizedAxisCoeffs() const;
    LineParams getOptimizedAxisParams() const;

    // 新增：获取最后一次评估结果
    AssessmentResult getLastAssessmentResult() const { return last_assessment_result_; }

   

private:
    // 核心算法函数
    LineParams optimizeLineParameters(); // 优化直线参数（4自由度）
    double computeDistanceToLine(const Eigen::Vector3f& point, const LineParams& line);
    double objectiveFunction(const LineParams& line); // 目标函数

    // 优化算法实现
    LineParams levenbergMarquardtOptimization(); // LM算法优化
    LineParams randomSearchOptimization();       // 随机搜索优化

    // 评估函数
    AssessmentResult computeAssessmentMetrics(const LineParams& line);

    // 工具函数
    Eigen::Vector3f computeCentroid() const;
    bool isValidPointCloud() const;
    void printDebugInfo(const std::string& message) const;
    bool isPointValid(const pcl::PointXYZ& point) const;

    // 直线参数工具
    LineParams initializeLineParameters(); // 初始化直线参数
    LineParams perturbLineParameters(const LineParams& line, float magnitude); // 参数扰动

    // 新增：生成热力图点云
    void generateHeatMapCloud(const LineParams& line);

    // 新增：热力图颜色映射函数
    void getColorForDistance(double distance, uint8_t& r, uint8_t& g, uint8_t& b) const;
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_;
    std::vector<double> distance_map_;

    double design_radius_;
    double tolerance_;
    int max_iterations_;
    bool verbose_;

    // 新增：外部提供的初始直线参数
    LineParams external_initial_line_;
    bool use_external_initial_line_;

    // 新增：保存最后一次评估结果
    AssessmentResult last_assessment_result_;
    LineParams last_optimized_line_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud_;
    double min_distance_;
    double max_distance_;
};