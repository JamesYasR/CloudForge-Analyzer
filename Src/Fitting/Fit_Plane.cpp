#include "Fitting/Fit_Plane.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace {
class CancellablePlaneModel : public pcl::SampleConsensusModelPlane<pcl::PointXYZ>
{
public:
    CancellablePlaneModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::function<bool()> keepRunning)
        : pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud), keepRunning_(std::move(keepRunning)) {}
    void getSamples(int& iterations, pcl::Indices& samples) override
    {
        if (!keepRunning_()) { samples.clear(); return; }
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::getSamples(iterations,samples);
    }
private:
    std::function<bool()> keepRunning_;
};
}


Fit_Plane::Fit_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC, const FitParams& params)
    : cloud_input(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>)
{
    PlaneFitThreshold = params.PlaneFitThreshold;
    MaxIterations = params.MaxIterations;

    if (InputC) *cloud_input = *InputC;
}

bool Fit_Plane::reportProgress(int current, const std::string& stage)
{
    if (isCancelled) return false;
    if (!progress_ || progress_(current, 100, stage)) return true;
    isCancelled = true;
    return false;
}

bool Fit_Plane::fitPlaneWithRANSAC() {
    if (cloud_input->empty()) {
        return false;
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new CancellablePlaneModel(cloud_input, [this]() { return reportProgress(10, "拟合平面"); }));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(PlaneFitThreshold);
    ransac.setMaxIterations(MaxIterations);
    if (!ransac.computeModel() || !reportProgress(85, "整理结果")) return false;

    ransac.getModelCoefficients(coeff_in);

    std::vector<int> ranSacInliers;
    ransac.getInliers(ranSacInliers);

    pcl::copyPointCloud(*cloud_input, ranSacInliers, *cloud_inliers);

    std::vector<unsigned char> mask(cloud_input->size(), 0);
    for (int index : ranSacInliers) mask[index]=1;
    std::vector<int> final_outliers;
    final_outliers.reserve(cloud_input->size()-ranSacInliers.size());
    for (int i=0; i<static_cast<int>(mask.size()); ++i) {
        if (i%4096==0 && !reportProgress(85+static_cast<int>(13.0*i/mask.size()), "整理结果")) return false;
        if (!mask[i]) final_outliers.push_back(i);
    }
    pcl::copyPointCloud(*cloud_input,final_outliers,*cloud_outliers);
    return true;
}

void Fit_Plane::compute() {
    coeff_in.resize(0);
    cloud_inliers->clear(); cloud_outliers->clear();
    message.clear(); isCancelled=false;
    auto fail = [this]() {
        coeff_in.resize(0);
        cloud_inliers->clear(); cloud_outliers->clear();
        message=isCancelled ? "平面拟合已取消" : "不可拟合: 未能获得有效的平面";
    };
    if (!reportProgress(0, "准备拟合")) { fail(); return; }
    if (cloud_input->size()<3 || !std::isfinite(PlaneFitThreshold) || PlaneFitThreshold<=0 || MaxIterations<1) { fail(); return; }
    for (size_t i=0;i<cloud_input->size();++i) {
        if (i%4096==0 && !reportProgress(1, "检查点云")) { fail(); return; }
        const auto& p=cloud_input->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) { fail(); return; }
    }
    if (!fitPlaneWithRANSAC() || coeff_in.size()<4
        || !coeff_in.allFinite() || coeff_in.head<3>().squaredNorm()==0 || cloud_inliers->empty()) { fail(); return; }

    // 归一化平面法向量
    Eigen::Vector3f normal(coeff_in[0], coeff_in[1], coeff_in[2]);
    float norm = normal.norm();
    if (norm > 0) {
        coeff_in[0] /= norm;
        coeff_in[1] /= norm;
        coeff_in[2] /= norm;
        coeff_in[3] /= norm;
    }

    // 计算RMS误差
    float rmse = ComputeRMSE();

    // 构建信息消息
    std::stringstream msg;
    msg << "=== 平面拟合结果 ===\n";
    msg << "输入点数: " << cloud_input->size() << "\n";
    msg << "平面外点数: " << cloud_outliers->size() << " (" << Get_Outliers_Percentage() << "%)\n";
    msg << "平面内点数（拟合结果）: " << cloud_inliers->size() << " (" << Get_Inliers_Percentage() << "%)\n";
    msg << "\n平面方程: " << std::fixed << std::setprecision(6)
        << coeff_in[0] << "x + "
        << coeff_in[1] << "y + "
        << coeff_in[2] << "z + "
        << coeff_in[3] << " = 0\n";
    msg << "法向量: (" << coeff_in[0] << ", " << coeff_in[1] << ", " << coeff_in[2] << ")\n";
    msg << "\n质量指标:\n";
    msg << "RMS误差: " << std::setprecision(4) << rmse << " mm\n";

    message = msg.str();
    if (!reportProgress(100, "完成拟合")) fail();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Plane::Get_Inliers() {
    return cloud_inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Plane::Get_Outliers() {
    return cloud_outliers;
}

Eigen::VectorXf Fit_Plane::Get_Coeff_in() {
    return coeff_in;
}

float Fit_Plane::ComputeRMSE() {
    if (coeff_in.size() < 4 || cloud_inliers->empty()) {
        return -1.0f;
    }

    float A = coeff_in[0];
    float B = coeff_in[1];
    float C = coeff_in[2];
    float D = coeff_in[3];

    float total_squared_error = 0.0f;
    int num_points = cloud_inliers->size();

    // 串行版本，不使用OpenMP
    for (int i = 0; i < num_points; ++i) {
        const auto& point = cloud_inliers->points[i];
        float distance = std::abs(A * point.x + B * point.y + C * point.z + D);
        total_squared_error += distance * distance;
    }

    return std::sqrt(total_squared_error / num_points);
}

float Fit_Plane::ComputePlanarityScore() {
    if (cloud_inliers->empty()) {
        return 0.0f;
    }

    float rmse = ComputeRMSE();
    if (rmse <= 0) return 0.0f;

    // 基于RMS误差计算平面度评分（0-100）
    // RMS误差越小，评分越高
    float max_acceptable_rmse = PlaneFitThreshold * 5.0f;
    float score = 100.0f * (1.0f - std::min(rmse / max_acceptable_rmse, 1.0f));

    return std::max(0.0f, std::min(100.0f, score));
}

float Fit_Plane::Get_Inliers_Percentage() {
    if (cloud_input->empty()) return 0.0f;
    return (static_cast<float>(cloud_inliers->size()) / static_cast<float>(cloud_input->size())) * 100.0f;
}

float Fit_Plane::Get_Outliers_Percentage() {
    if (cloud_input->empty()) return 0.0f;
    return (static_cast<float>(cloud_outliers->size()) / static_cast<float>(cloud_input->size())) * 100.0f;
}
