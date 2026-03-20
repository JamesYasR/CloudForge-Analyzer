#pragma once
#include "config/pcl114.h"
#include <memory>

class MeasurePlanarity
{
public:
    struct PlaneParams {
        Eigen::Vector4f coefficients;  // [A, B, C, D] 平面方程参数

        PlaneParams() : coefficients(Eigen::Vector4f::Zero()) {}

        PlaneParams(const Eigen::Vector4f& coeffs) : coefficients(coeffs) {
            // 归一化
            Eigen::Vector3f normal = coeffs.head<3>();
            float norm = normal.norm();
            if (norm > 0) {
                coefficients /= norm;
            }
        }

        // 从pcl::ModelCoefficients构造
        PlaneParams(const pcl::ModelCoefficients::Ptr coeffs) {
            if (coeffs && coeffs->values.size() >= 4) {
                coefficients[0] = coeffs->values[0];
                coefficients[1] = coeffs->values[1];
                coefficients[2] = coeffs->values[2];
                coefficients[3] = coeffs->values[3];

                // 归一化
                Eigen::Vector3f normal = coefficients.head<3>();
                float norm = normal.norm();
                if (norm > 0) {
                    coefficients /= norm;
                }
            }
        }

        // 获取平面法向量
        Eigen::Vector3f getNormal() const {
            return coefficients.head<3>();
        }

        // 获取平面系数数组
        std::vector<float> toCoeffs() const {
            return { coefficients[0], coefficients[1], coefficients[2], coefficients[3] };
        }
    };

    struct AssessmentResult {
        double min_distance;           // 最小距离
        double max_distance;           // 最大距离
        double mean_distance;          // 平均距离
        double rms_distance;           // 均方根距离
        double std_distance;           // 距离标准差
        double peak_to_valley;         // 峰谷值 (最大-最小)

        std::string assessment_message; // 评估结果消息

        AssessmentResult() : min_distance(0.0), max_distance(0.0),
            mean_distance(0.0), rms_distance(0.0),
            std_distance(0.0), peak_to_valley(0.0) {
        }
    };

    // 获取带热力图颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getHeatMapCloud() const { return heatmap_cloud_; }

    // 获取距离范围用于颜色映射
    void getDistanceRange(double& min_distance, double& max_distance) const {
        min_distance = min_distance_;
        max_distance = max_distance_;
    }

    MeasurePlanarity();
    ~MeasurePlanarity();

    // 设置输入参数
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setPlaneParameters(const PlaneParams& plane_params);
    void setPlaneParameters(const pcl::ModelCoefficients::Ptr coeffs);
    void setPlaneParameters(const Eigen::Vector4f& coeffs);

    // 执行平面度评估
    AssessmentResult evaluatePlanarity();

    // 获取距离数据
    std::vector<double> getDistanceMap() const { return distance_map_; }

    // 获取最后一次评估结果
    AssessmentResult getLastAssessmentResult() const { return last_assessment_result_; }

private:
    // 核心计算函数
    double computeDistanceToPlane(const pcl::PointXYZ& point) const;
    void computeAssessmentMetrics();

    // 热力图相关函数
    void generateHeatMapCloud();
    void getColorForDistance(double distance, uint8_t& r, uint8_t& g, uint8_t& b) const;

    // 工具函数
    bool isValidPointCloud() const;
    void printDebugInfo(const std::string& message) const;
    bool isPointValid(const pcl::PointXYZ& point) const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    PlaneParams plane_params_;

    std::vector<double> distance_map_;
    AssessmentResult last_assessment_result_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud_;
    double min_distance_;
    double max_distance_;

    bool verbose_;
};