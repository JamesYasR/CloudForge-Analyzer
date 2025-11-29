#include "Measure/MeasureHeight.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
#include <limits>
#include <numeric>

MeasureHeight::MeasureHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr measure_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud)
    : measure_cloud_(measure_cloud),
    reference_cloud_(reference_cloud),
    plane_coeffs_(new pcl::ModelCoefficients())
{
}

bool MeasureHeight::measure() {
    if (!measure_cloud_ || measure_cloud_->empty()) return false;
    if (!reference_cloud_ || reference_cloud_->empty()) return false;

    if (!fitPlane()) return false;

    computeDistances();
    measured_ = true;
    return true;
}

bool MeasureHeight::fitPlane() {
    // 使用 RANSAC 拟合平面
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // 默认阈值，可按需调整
    seg.setMaxIterations(1000);
    seg.setInputCloud(reference_cloud_);

    pcl::ModelCoefficients coefficients;
    seg.segment(*inliers, coefficients);
    if (inliers->indices.empty()) {
        return false;
    }

    plane_coeffs_->values = coefficients.values;
    return true;
}

void MeasureHeight::computeDistances() {
    distances_.clear();
    if (!plane_coeffs_ || plane_coeffs_->values.size() < 4) {
        // 未拟合，置为 0
        max_dist_ = min_dist_ = mean_dist_ = 0.0;
        return;
    }

    double a = plane_coeffs_->values[0];
    double b = plane_coeffs_->values[1];
    double c = plane_coeffs_->values[2];
    double d = plane_coeffs_->values[3];
    double denom = std::sqrt(a * a + b * b + c * c);
    if (denom == 0.0) denom = 1.0;

    distances_.reserve(measure_cloud_->points.size());
    for (const auto& p : measure_cloud_->points) {
        double num = a * p.x + b * p.y + c * p.z + d;
        double signed_dist = num / denom;
        double abs_dist = std::fabs(signed_dist);
        distances_.push_back(abs_dist);
    }

    if (distances_.empty()) {
        max_dist_ = min_dist_ = mean_dist_ = 0.0;
        return;
    }

    double sum = 0.0;
    double maxv = std::numeric_limits<double>::lowest();
    double minv = std::numeric_limits<double>::max();
    for (double v : distances_) {
        sum += v;
        if (v > maxv) maxv = v;
        if (v < minv) minv = v;
    }
    max_dist_ = maxv;
    min_dist_ = minv;
    mean_dist_ = sum / static_cast<double>(distances_.size());
}

double MeasureHeight::GetMaxDistance() const {
    return max_dist_;
}

double MeasureHeight::GetMinDistance() const {
    return min_dist_;
}

double MeasureHeight::GetMeanDistance() const {
    return mean_dist_;
}

pcl::ModelCoefficients::Ptr MeasureHeight::GetPlaneCoefficients() const {
    return plane_coeffs_;
}