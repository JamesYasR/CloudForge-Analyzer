#include "Fitting/Fit_Line.h"
#include "Dialog/ParamDialog_FittingLine.h"
#include <omp.h>
#include <pcl/common/pca.h>

Fit_Line::Fit_Line(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC)
    : paramDialog(new ParamDialog_FittingLine())
    , cloud_input(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>)
{
    bool ok1, ok2;

    // 从对话框获取参数
    if (paramDialog->exec() == QDialog::Accepted) {
        QString param1 = paramDialog->getParams()[0]; // 距离阈值
        QString param2 = paramDialog->getParams()[1]; // 最大迭代次数

        DistanceThreshold = param1.toFloat(&ok1);
        MaxIterations = param2.toInt(&ok2);

        if (!ok1 || !ok2) {
            qDebug() << "无效参数";
            return;
        }
    }
    else {
        qDebug() << "取消操作";
        return;
    }

    *cloud_input = *InputC;
    if (cloud_input->empty()) {
        qDebug() << "点云为空";
        return;
    }

    Proc(); // 执行拟合
}

Fit_Line::~Fit_Line() = default;

void Fit_Line::Proc() {
    // 创建直线模型
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_input)
    );

    // 创建RANSAC对象
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(DistanceThreshold);
    ransac.setMaxIterations(MaxIterations);

    // 执行拟合
    ransac.computeModel();
    ransac.getModelCoefficients(coeff_in);

    // 获取内点索引
    std::vector<int> inlierIndices;
    ransac.getInliers(inlierIndices);

    // 提取内点
    pcl::copyPointCloud(*cloud_input, inlierIndices, *cloud_inliers);

    // === 多线程优化：外点提取 ===
    cloud_outliers->clear();
    int total_points = static_cast<int>(cloud_input->size());

    if (total_points > 10000) {
        std::unordered_set<int> inlierSet(inlierIndices.begin(), inlierIndices.end());
        std::vector<std::vector<pcl::PointXYZ>> thread_outliers(omp_get_max_threads());

#pragma omp parallel for
        for (int i = 0; i < total_points; ++i) {
            int thread_id = omp_get_thread_num();
            if (inlierSet.find(i) == inlierSet.end()) {
                thread_outliers[thread_id].push_back((*cloud_input)[i]);
            }
        }

        for (const auto& outliers : thread_outliers) {
            cloud_outliers->insert(cloud_outliers->end(), outliers.begin(), outliers.end());
        }
    }
    else {
        std::unordered_set<int> inlierSet(inlierIndices.begin(), inlierIndices.end());
        for (size_t i = 0; i < cloud_input->size(); ++i) {
            if (inlierSet.find(i) == inlierSet.end()) {
                cloud_outliers->push_back((*cloud_input)[i]);
            }
        }
    }

    // === 修正：基于实际边界计算起始点和终点 ===
    if (!Get_Inliers()->empty()) {
        Eigen::Vector3f line_point(Get_Coeff_in()[0], Get_Coeff_in()[1], Get_Coeff_in()[2]);
        Eigen::Vector3f line_direction(Get_Coeff_in()[3], Get_Coeff_in()[4], Get_Coeff_in()[5]);
        line_direction.normalize();

        // 计算投影参数范围
        float min_param = std::numeric_limits<float>::max();
        float max_param = std::numeric_limits<float>::min();

        int inlier_size = static_cast<int>(Get_Inliers()->size());
        if (inlier_size > 5000) {
#pragma omp parallel
            {
                float local_min = std::numeric_limits<float>::max();
                float local_max = std::numeric_limits<float>::min();

#pragma omp for
                for (int i = 0; i < inlier_size; ++i) {
                    const auto& point = Get_Inliers()->points[i];
                    Eigen::Vector3f pt_vec = point.getVector3fMap() - line_point;
                    float param = pt_vec.dot(line_direction);

                    if (param < local_min) local_min = param;
                    if (param > local_max) local_max = param;
                }

#pragma omp critical
                {
                    if (local_min < min_param) min_param = local_min;
                    if (local_max > max_param) max_param = local_max;
                }
            }
        }
        else {
            for (const auto& point : *Get_Inliers()) {
                Eigen::Vector3f pt_vec = point.getVector3fMap() - line_point;
                float param = pt_vec.dot(line_direction);

                if (param < min_param) min_param = param;
                if (param > max_param) max_param = param;
            }
        }

        // === 关键修正：动态安全边界 ===
        float actual_length = max_param - min_param;
        float safety_margin = actual_length * 0.15f;  // 实际长度的15%
        safety_margin = std::max(safety_margin, 0.08f);  // 至少8cm

        min_param -= safety_margin;
        max_param += safety_margin;

        // 计算起点和终点
        start_point.getVector3fMap() = line_point + min_param * line_direction;
        end_point.getVector3fMap() = line_point + max_param * line_direction;

    }
    else {
        // 备用方案
        Eigen::Vector3f line_point(Get_Coeff_in()[0], Get_Coeff_in()[1], Get_Coeff_in()[2]);
        Eigen::Vector3f line_direction(Get_Coeff_in()[3], Get_Coeff_in()[4], Get_Coeff_in()[5]);
        line_direction.normalize();

        start_point.getVector3fMap() = line_point - 0.5f * line_direction;
        end_point.getVector3fMap() = line_point + 0.5f * line_direction;
    }

    // 输出调试信息
    qDebug() << "直线上一点: (" << coeff_in[0] << ", " << coeff_in[1] << ", " << coeff_in[2] << ")";
    qDebug() << "方向向量: (" << coeff_in[3] << ", " << coeff_in[4] << ", " << coeff_in[5] << ")";
    qDebug() << "内点数量: " << cloud_inliers->size() << ", 外点数量: " << cloud_outliers->size();
    qDebug() << "起点: (" << start_point.x << ", " << start_point.y << ", " << start_point.z << ")";
    qDebug() << "终点: (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Line::Get_Inliers() {
    return cloud_inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Line::Get_Outliers() {
    return cloud_outliers;
}

Eigen::VectorXf Fit_Line::Get_Coeff_in() {
    return coeff_in;
}