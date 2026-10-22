#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <functional>
#include <string>
#include <utility>

class Fit_Plane {
public:
    struct FitParams {
        float PlaneFitThreshold = 1.0f;
        int MaxIterations = 1000;
    };

    Fit_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC, const FitParams& params);
    using ProgressCallback = std::function<bool(int, int, const std::string&)>;
    void setProgressCallback(ProgressCallback callback) { progress_ = std::move(callback); }
    void compute();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Outliers();
    Eigen::VectorXf Get_Coeff_in();
    float Get_Inliers_Percentage();
    float Get_Outliers_Percentage();
    float ComputeRMSE();
    float ComputePlanarityScore();

    bool isCancelled = false;
    std::string message;

private:
    float PlaneFitThreshold;
    int MaxIterations;
    Eigen::VectorXf coeff_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;
    ProgressCallback progress_;
    bool reportProgress(int current, const std::string& stage);
    bool fitPlaneWithRANSAC();
};
