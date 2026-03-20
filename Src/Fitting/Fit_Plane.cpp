#include "Fitting/Fit_Plane.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

Fit_Plane::Fit_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC)
    : dialog(new ParamDialog_FittingPlane())
    , cloud_input(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_anomaly(new pcl::PointCloud<pcl::PointXYZ>)
    , coeff_in(Eigen::Vector4f::Zero())  // 添加初始化
{
    bool ok1, ok2, ok3,ok4;

    if (dialog->exec() != QDialog::Accepted) {
        isCancelled = true;
        return;
    }
    else {
        QString param1 = dialog->getParams()[0];  // 局部拟合半径
        QString param2 = dialog->getParams()[1];  // 距离阈值
        QString param3 = dialog->getParams()[2];  // 最大迭代次数
        QString param4 = dialog->getParams()[3];  // 最大迭代次数

        LocalRadius = param1.toFloat(&ok1);
        AnomalyThreshold = param2.toFloat(&ok2);    // 读取第二个参数作为异常点阈值
        PlaneFitThreshold = param3.toFloat(&ok3);   // 读取第三个参数作为平面拟合阈值
        MaxIterations = param4.toInt(&ok4);         // 读取第四个参数

        if (!ok1 || !ok2 || !ok3) {
            qDebug() << "无效数字";
            isCancelled = true;
            return;
        }
    }

    *cloud_input = *InputC;
    if (cloud_input->empty()) {
        qDebug() << "点云为空";
        isCancelled = true;
        return;
    }

    Proc();
}

Fit_Plane::~Fit_Plane() = default;

void Fit_Plane::detectAnomalyPoints() {
    if (LocalRadius <= 0 || AnomalyThreshold <= 0) {
        // 如果禁用异常点检测，所有点都参与后续拟合
        *cloud_outliers = *cloud_input;
        cloud_inliers->clear();
        cloud_anomaly->clear();
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_input);

    std::vector<int> normal_indices;  // 存放正常点索引
    std::vector<int> anomaly_indices; // 存放异常点索引

    for (int i = 0; i < static_cast<int>(cloud_input->size()); ++i) {
        const auto& point = cloud_input->points[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(point, LocalRadius, pointIdxRadiusSearch,
            pointRadiusSquaredDistance) > 3) {
            // 有足够邻居点，计算局部平面
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : pointIdxRadiusSearch) {
                local_cloud->push_back(cloud_input->points[idx]);
            }

            // 局部平面拟合
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(AnomalyThreshold); // 使用异常点阈值进行局部拟合
            seg.setInputCloud(local_cloud);
            seg.segment(*inliers, *coefficients);

            if (coefficients->values.size() >= 4) {
                // 计算当前点到这个局部拟合平面的距离
                float A = coefficients->values[0];
                float B = coefficients->values[1];
                float C = coefficients->values[2];
                float D = coefficients->values[3];
                float distance_to_local_plane = std::abs(A * point.x + B * point.y + C * point.z + D);

                // 判断：如果距离大于异常点阈值，则标记为异常
                if (distance_to_local_plane > AnomalyThreshold) {
                    anomaly_indices.push_back(i);
                }
                else {
                    normal_indices.push_back(i);
                }
            }
            else {
                // 局部拟合失败，保守处理，视为正常点
                normal_indices.push_back(i);
            }
        }
        else {
            // 孤立点，保守处理，视为正常点（或根据需求改为异常点）
            normal_indices.push_back(i);
        }
    }

    // 提取点云
    pcl::copyPointCloud(*cloud_input, normal_indices, *cloud_outliers);
    pcl::copyPointCloud(*cloud_input, anomaly_indices, *cloud_anomaly);

    qDebug() << "异常点检测完成，正常点（将参与拟合）:" << cloud_outliers->size()
        << "，异常点（已排除）:" << cloud_anomaly->size();
}

void Fit_Plane::fitPlaneWithRANSAC() {
    if (cloud_outliers->empty()) {
        qDebug() << "没有可用于拟合的点";
        return;
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_outliers));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(PlaneFitThreshold);
    ransac.setMaxIterations(MaxIterations);
    ransac.computeModel();

    ransac.getModelCoefficients(coeff_in);

    std::vector<int> ranSacInliers;
    ransac.getInliers(ranSacInliers);

    pcl::copyPointCloud(*cloud_outliers, ranSacInliers, *cloud_inliers);

    // 重新计算外点（排除异常点后的外点）
    std::vector<int> final_outliers;
    for (int i = 0; i < static_cast<int>(cloud_outliers->size()); ++i) {
        if (std::find(ranSacInliers.begin(), ranSacInliers.end(), i) == ranSacInliers.end()) {
            final_outliers.push_back(i);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_outliers, final_outliers, *temp_outliers);
    *cloud_outliers = *temp_outliers;
}

void Fit_Plane::Proc() {

    //*cloud_outliers = *cloud_input;
    //cloud_anomaly->clear();  // 清空异常点集合
    detectAnomalyPoints();
    // 步骤2: 平面拟合
    fitPlaneWithRANSAC();

    if (coeff_in.size() < 4) {
        qDebug() << "平面拟合失败：未能获得有效的模型系数";
        cloud_inliers->clear();
        *cloud_outliers = *cloud_input;
        return;
    }

    // 归一化平面法向量
    Eigen::Vector3f normal(coeff_in[0], coeff_in[1], coeff_in[2]);
    float norm = normal.norm();
    if (norm > 0) {
        coeff_in[0] /= norm;
        coeff_in[1] /= norm;
        coeff_in[2] /= norm;
        coeff_in[3] /= norm;
    }

    qDebug() << "平面方程系数: A=" << coeff_in[0]
        << ", B=" << coeff_in[1]
        << ", C=" << coeff_in[2]
        << ", D=" << coeff_in[3];

    // 计算RMS误差
    float rmse = ComputeRMSE();
    float planarity_score = ComputePlanarityScore();

    // 构建信息消息
    std::stringstream msg;
    msg << "=== 平面拟合结果 ===\n";
    msg << "输入点数: " << cloud_input->size() << "\n";
    msg << "异常点数（已排除）: " << cloud_anomaly->size() << " (" << Get_Anomaly_Percentage() << "%)\n";
    msg << "参与拟合点数: " << cloud_outliers->size() << " (" << Get_Outliers_Percentage() + Get_Inliers_Percentage() << "%)\n";
    msg << "平面内点数（拟合结果）: " << cloud_inliers->size() << " (" << Get_Inliers_Percentage() << "%)\n";
    msg << "\n平面方程: " << std::fixed << std::setprecision(6)
        << coeff_in[0] << "x + "
        << coeff_in[1] << "y + "
        << coeff_in[2] << "z + "
        << coeff_in[3] << " = 0\n";
    msg << "法向量: (" << coeff_in[0] << ", " << coeff_in[1] << ", " << coeff_in[2] << ")\n";
    msg << "\n质量指标:\n";
    msg << "RMS误差: " << std::setprecision(4) << rmse << " mm\n";
    msg << "平面度评分: " << std::setprecision(2) << planarity_score << "/100\n";

    message = msg.str();
    qDebug() << QString::fromStdString(message);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Plane::Get_Inliers() {
    return cloud_inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Plane::Get_Outliers() {
    return cloud_outliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Plane::Get_AnomalyPoints() {
    return cloud_anomaly;
}

Eigen::VectorXf Fit_Plane::Get_Coeff_in() {  // 注意：返回类型应该是Eigen::Vector4f
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

float Fit_Plane::Get_Anomaly_Percentage() {
    if (cloud_input->empty()) return 0.0f;
    return (static_cast<float>(cloud_anomaly->size()) / static_cast<float>(cloud_input->size())) * 100.0f;
}