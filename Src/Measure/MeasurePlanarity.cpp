#include "Measure/MeasurePlanarity.h"
#include <pcl/common/centroid.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <omp.h>

MeasurePlanarity::MeasurePlanarity()
    : verbose_(true)
    , min_distance_(0.0)
    , max_distance_(0.0)
{
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    heatmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    last_assessment_result_ = AssessmentResult();
}

MeasurePlanarity::~MeasurePlanarity()
{
}

void MeasurePlanarity::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud && !cloud->empty()) {
        input_cloud_ = cloud;
        if (verbose_) {
            std::cout << "[MeasurePlanarity] 点云设置完成，点数: " << input_cloud_->size() << std::endl;
        }
    }
    else {
        if (verbose_) {
            std::cout << "[MeasurePlanarity] 错误: 输入点云为空或无效" << std::endl;
        }
    }
}

void MeasurePlanarity::setPlaneParameters(const PlaneParams& plane_params)
{
    plane_params_ = plane_params;
    if (verbose_) {
        std::cout << "[MeasurePlanarity] 平面参数设置: A=" << plane_params_.coefficients[0]
            << ", B=" << plane_params_.coefficients[1]
            << ", C=" << plane_params_.coefficients[2]
            << ", D=" << plane_params_.coefficients[3] << std::endl;
    }
}

void MeasurePlanarity::setPlaneParameters(const pcl::ModelCoefficients::Ptr coeffs)
{
    if (coeffs && coeffs->values.size() >= 4) {
        plane_params_ = PlaneParams(coeffs);
        if (verbose_) {
            std::cout << "[MeasurePlanarity] 从ModelCoefficients设置平面参数" << std::endl;
        }
    }
    else {
        if (verbose_) {
            std::cout << "[MeasurePlanarity] 错误: ModelCoefficients无效" << std::endl;
        }
    }
}

void MeasurePlanarity::setPlaneParameters(const Eigen::Vector4f& coeffs)
{
    plane_params_ = PlaneParams(coeffs);
    if (verbose_) {
        std::cout << "[MeasurePlanarity] 从Vector4f设置平面参数" << std::endl;
    }
}

double MeasurePlanarity::computeDistanceToPlane(const pcl::PointXYZ& point) const
{
    const Eigen::Vector4f& coeffs = plane_params_.coefficients;
    return std::abs(coeffs[0] * point.x + coeffs[1] * point.y +
        coeffs[2] * point.z + coeffs[3]);
}

bool MeasurePlanarity::isPointValid(const pcl::PointXYZ& point) const
{
    return pcl::isFinite(point) &&
        !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z);
}

void MeasurePlanarity::computeAssessmentMetrics()
{
    distance_map_.clear();

    if (!isValidPointCloud()) {
        return;
    }

    int num_points = static_cast<int>(input_cloud_->size());
    distance_map_.reserve(num_points);

    double sum_distance = 0.0;
    double sum_squared_distance = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    double max_dist = std::numeric_limits<double>::lowest();

    // 第一阶段：并行计算所有点的距离
    std::vector<double> distances(num_points, 0.0);
    std::vector<bool> valid_points(num_points, false);

#pragma omp parallel for reduction(min:min_dist) reduction(max:max_dist)
    for (int i = 0; i < num_points; ++i) {
        const auto& point = input_cloud_->points[i];
        if (!isPointValid(point)) {
            valid_points[i] = false;
            continue;
        }

        double distance = computeDistanceToPlane(point);
        distances[i] = distance;
        valid_points[i] = true;

        if (distance < min_dist) min_dist = distance;
        if (distance > max_dist) max_dist = distance;
    }

    // 第二阶段：串行统计
    int valid_count = 0;
    for (int i = 0; i < num_points; ++i) {
        if (!valid_points[i]) continue;

        double distance = distances[i];
        distance_map_.push_back(distance);
        sum_distance += distance;
        sum_squared_distance += distance * distance;
        valid_count++;
    }

    if (valid_count > 0) {
        last_assessment_result_.min_distance = min_dist;
        last_assessment_result_.max_distance = max_dist;
        last_assessment_result_.mean_distance = sum_distance / valid_count;
        last_assessment_result_.rms_distance = std::sqrt(sum_squared_distance / valid_count);
        last_assessment_result_.peak_to_valley = max_dist - min_dist;

        // 计算标准差
        double sum_variance = 0.0;
        for (double distance : distance_map_) {
            double diff = distance - last_assessment_result_.mean_distance;
            sum_variance += diff * diff;
        }
        last_assessment_result_.std_distance = std::sqrt(sum_variance / valid_count);
    }

    min_distance_ = min_dist;
    max_distance_ = max_dist;
}

void MeasurePlanarity::getColorForDistance(double distance, uint8_t& r, uint8_t& g, uint8_t& b) const
{
    // 将距离映射到0-1的范围
    double normalized_distance = 0.0;
    if (max_distance_ > min_distance_) {
        normalized_distance = (distance - min_distance_) / (max_distance_ - min_distance_);
    }

    // 限制在0-1范围内
    normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));

    // 绿色(0.5) -> 红色(1.0)的渐变
    if (normalized_distance < 0.5) {
        // 绿色到黄色：红色递增，绿色保持255
        r = static_cast<uint8_t>(255 * (normalized_distance * 2));
        g = 255;
        b = 0;
    }
    else {
        // 黄色到红色：红色保持255，绿色递减
        r = 255;
        g = static_cast<uint8_t>(255 * (2.0 - normalized_distance * 2));
        b = 0;
    }
}

void MeasurePlanarity::generateHeatMapCloud()
{
    heatmap_cloud_->clear();
    heatmap_cloud_->width = input_cloud_->width;
    heatmap_cloud_->height = input_cloud_->height;
    heatmap_cloud_->is_dense = input_cloud_->is_dense;
    heatmap_cloud_->points.resize(input_cloud_->size());

    int num_points = static_cast<int>(input_cloud_->size());

#pragma omp parallel for
    for (int i = 0; i < num_points; ++i) {
        const auto& point = input_cloud_->points[i];
        heatmap_cloud_->points[i].x = point.x;
        heatmap_cloud_->points[i].y = point.y;
        heatmap_cloud_->points[i].z = point.z;

        if (isPointValid(point)) {
            double distance = computeDistanceToPlane(point);

            uint8_t r, g, b;
            getColorForDistance(distance, r, g, b);

            heatmap_cloud_->points[i].r = r;
            heatmap_cloud_->points[i].g = g;
            heatmap_cloud_->points[i].b = b;
        }
        else {
            // 无效点处理为灰色
            heatmap_cloud_->points[i].r = 128;
            heatmap_cloud_->points[i].g = 128;
            heatmap_cloud_->points[i].b = 128;
        }
    }
}

MeasurePlanarity::AssessmentResult MeasurePlanarity::evaluatePlanarity()
{
    AssessmentResult result;

    if (!isValidPointCloud()) {
        result.assessment_message = "错误: 输入点云无效";
        return result;
    }

    if (std::abs(plane_params_.coefficients.head<3>().norm() - 1.0f) > 1e-6) {
        result.assessment_message = "错误: 平面参数未归一化或无效";
        return result;
    }

    if (verbose_) {
        std::cout << "[MeasurePlanarity] 开始平面度评估..." << std::endl;
        std::cout << "[MeasurePlanarity] 点云点数: " << input_cloud_->size() << std::endl;
    }

    try {
        // 计算评估指标
        computeAssessmentMetrics();

        // 生成热力图
        generateHeatMapCloud();

        // 构建结果消息
        std::stringstream message;
        message << "=== 平面度评估报告 ===\n";
        message << "点云点数: " << input_cloud_->size() << "\n\n";

        message << "平面方程: " << std::fixed << std::setprecision(6)
            << plane_params_.coefficients[0] << "x + "
            << plane_params_.coefficients[1] << "y + "
            << plane_params_.coefficients[2] << "z + "
            << plane_params_.coefficients[3] << " = 0\n";
        message << "法向量: (" << plane_params_.coefficients[0] << ", "
            << plane_params_.coefficients[1] << ", "
            << plane_params_.coefficients[2] << ")\n\n";

        message << "距离统计指标:\n";
        message << "最小距离: " << std::setprecision(4) << last_assessment_result_.min_distance << " mm\n";
        message << "最大距离: " << std::setprecision(4) << last_assessment_result_.max_distance << " mm\n";
        message << "峰谷值(PV): " << std::setprecision(4) << last_assessment_result_.peak_to_valley << " mm\n";
        message << "平均距离: " << std::setprecision(4) << last_assessment_result_.mean_distance << " mm\n";
        message << "RMS距离: " << std::setprecision(4) << last_assessment_result_.rms_distance << " mm\n";
        message << "距离标准差: " << std::setprecision(4) << last_assessment_result_.std_distance << " mm\n\n";

        message << "热力图颜色范围: " << min_distance_ << " mm (蓝) 到 "
            << max_distance_ << " mm (红)";

        result = last_assessment_result_;
        result.assessment_message = message.str();

    }
    catch (const std::exception& e) {
        result.assessment_message = "评估过程出错: " + std::string(e.what());
    }

    if (verbose_) {
        std::cout << result.assessment_message << std::endl;
    }

    return result;
}

bool MeasurePlanarity::isValidPointCloud() const
{
    return input_cloud_ && !input_cloud_->empty();
}

void MeasurePlanarity::printDebugInfo(const std::string& message) const
{
    if (verbose_) {
        std::cout << "[MeasurePlanarity] " << message << std::endl;
    }
}