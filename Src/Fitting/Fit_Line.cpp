#include "Fitting/Fit_Line.h"
#include "Dialog/ParamDialog_FittingLine.h"

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
    ransac.getModelCoefficients(coeff_in); // 获取模型系数

    // 获取内点索引
    std::vector<int> inlierIndices;
    ransac.getInliers(inlierIndices);

    // 提取内点
    pcl::copyPointCloud(*cloud_input, inlierIndices, *cloud_inliers);

    // 提取外点
    cloud_outliers->clear();
    std::unordered_set<int> inlierSet(inlierIndices.begin(), inlierIndices.end());
    for (size_t i = 0; i < cloud_input->size(); ++i) {
        if (inlierSet.find(i) == inlierSet.end()) {
            cloud_outliers->push_back((*cloud_input)[i]);
        }
    }
    float min_param = std::numeric_limits<float>::max();
    float max_param = std::numeric_limits<float>::min();

    Eigen::Vector3f line_point(Get_Coeff_in()[0], Get_Coeff_in()[1], Get_Coeff_in()[2]);
    Eigen::Vector3f line_direction(Get_Coeff_in()[3], Get_Coeff_in()[4], Get_Coeff_in()[5]);
    line_direction.normalize();

    for (const auto& point : *Get_Inliers()) {
        Eigen::Vector3f pt_vec = point.getVector3fMap() - line_point;
        float param = pt_vec.dot(line_direction);

        if (param < min_param) min_param = param;
        if (param > max_param) max_param = param;
    }

    // 计算起点和终点
    start_point.getVector3fMap() = line_point + min_param * line_direction;
    end_point.getVector3fMap() = line_point + max_param * line_direction;
    // 输出直线参数（调试用）
    qDebug() << "直线上一点: (" << coeff_in[0] << ", " << coeff_in[1] << ", " << coeff_in[2] << ")";
    qDebug() << "方向向量: (" << coeff_in[3] << ", " << coeff_in[4] << ", " << coeff_in[5] << ")";
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