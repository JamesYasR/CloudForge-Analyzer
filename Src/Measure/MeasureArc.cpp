#include "Measure\MeasureArc.h"
#include "Dialog/ParamDialog_measureArc.h"
#include "Fitting/Fit_Cylinder.h"

MeasureArc::MeasureArc()
	: cloud(new pcl::PointCloud<pcl::PointXYZ>()),
   center(*(new Eigen::Vector3f())),
   axis_direction(*(new Eigen::Vector3f())),
   radius(0.0)
{

}
MeasureArc::~MeasureArc() = default;

double MeasureArc::calculateArcLength() {
    if (cloud->empty() || radius <= 0) {
        qDebug() << "1";
        return -1.0;
    }

    // 单位化轴线向量
    Eigen::Vector3f axis = axis_direction.normalized();

    // 步骤1：创建投影平面（垂直于圆柱轴线）
    Eigen::Vector4f plane_parameters;
    plane_parameters.head<3>() = axis.cast<float>();
    plane_parameters[3] = -center.dot(axis); // 平面方程：ax+by+cz+d=0 中的d

    // 步骤2：将点云投影到该平面
    PointCloud<PointXYZ>::Ptr projected_cloud(new PointCloud<PointXYZ>);
    ProjectInliers<PointXYZ> proj;
    proj.setModelType(SACMODEL_PLANE);
    proj.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    coeffs->values.resize(4);
    coeffs->values[0] = plane_parameters[0];
    coeffs->values[1] = plane_parameters[1];
    coeffs->values[2] = plane_parameters[2];
    coeffs->values[3] = plane_parameters[3];

    proj.setModelCoefficients(coeffs);
    proj.filter(*projected_cloud);

    if (projected_cloud->empty()) {
        qDebug() << "2";
        return -1.0;
    }

    // 步骤3：计算投影点到圆心的向量并计算角度
    std::vector<double> angles;
    const Eigen::Vector3f center_on_plane = center + axis * (-center.dot(axis));

    for (const auto& pt : *projected_cloud) {
        Eigen::Vector3f vec(pt.x - center_on_plane.x(),
            pt.y - center_on_plane.y(),
            pt.z - center_on_plane.z());

        // 可选：检查点是否在圆柱面上（误差容忍度）
        // if (std::fabs(vec.norm() - radius) > 0.1 * radius) {
        //     continue; // 跳过偏差太大的点
        // }

        // 归一化并计算角度 [0, 2π)
        vec.normalize();
        double angle = std::atan2(vec.dot(Eigen::Vector3f::UnitY()),
            vec.dot(Eigen::Vector3f::UnitX()));
        if (angle < 0) angle += 2 * M_PI;
        angles.push_back(angle);
    }

    if (angles.empty()) {
        qDebug() << "3";
        return -1.0;
    }

    // 步骤4：排序角度并计算角度跨度
    std::sort(angles.begin(), angles.end());

    // 计算相邻角度差并找到最大间隔
    double max_gap = 0.0;
    const double full_circle = 2 * M_PI;

    for (size_t i = 0; i < angles.size(); ++i) {
        double gap = 0.0;
        if (i == angles.size() - 1) {
            // 首尾角度间隙（处理0°/360°边界）
            gap = (angles.front() + full_circle) - angles.back();
        }
        else {
            gap = angles[i + 1] - angles[i];
        }

        if (gap > max_gap) max_gap = gap;
    }

    // 弧长 = 半径 × 跨越角度（总圆减去最大间隙）
    double arc_angle = full_circle - max_gap;
    return radius * arc_angle;
}

void MeasureArc::FitCylinder(PointCloud<PointXYZ>::Ptr inputcloud) {
	Fit_Cylinder fcy(inputcloud);
	cloud = fcy.Get_Inliers();
	center = fcy.Get_Coeff_in().head<3>();
	axis_direction = fcy.Get_Coeff_in().segment<3>(3);
    radius= fcy.Get_Coeff_in()[6];
}