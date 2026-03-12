#include "Measure/MeasureArc.h"
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkParametricFunctionSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkMath.h>
#include <vtkProperty.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <functional>
#include <QDebug>

MeasureArc::MeasureArc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::ModelCoefficients::Ptr cylinder_coeff,
    pcl::PointXYZ* specified_point)
    : m_inputCloud(cloud)
    , m_cylinderCoeff(cylinder_coeff)
    , m_specifiedPoint(specified_point)
    , m_sliceCloud(new pcl::PointCloud<pcl::PointXYZ>)
    , arcLength(0.0)
    , success(false)
    , message("Initialized.")
    , m_actualParamStart(0.0)
    , m_actualParamEnd(1.0)
    , m_hasVirtualPoints(false)
    ,dialog(new ParamDialogMeaArc())
{
    // 初始化成员变量
    m_axisPoint = Eigen::Vector3f::Zero();
    m_axisDirection = Eigen::Vector3f::Zero();
    m_cylinderRadius = 0.0;
    m_planeCenter = Eigen::Vector3f::Zero();
    m_planeX = Eigen::Vector3f::Zero();
    m_planeY = Eigen::Vector3f::Zero();

    if (dialog->exec() != QDialog::Accepted) {
        isCancelled = true;
        return;
    }
    else // 如果用户点击了"确定"
    {
        bool ok1, ok2, ok3, ok4;
        QString param1 = dialog->getParams()[0];
        QString param2 = dialog->getParams()[1];
        QString param3 = dialog->getParams()[2];
        QString param4 = dialog->getParams()[3];
        sliceThicknessFactor = param1.toDouble(&ok1);
        integrationTolerance = param2.toDouble(&ok2);
        downsampleTargetSize = param3.toInt(&ok3);
        virtualPointExtrapolation = param4.toDouble(&ok4);  // 新增：解析初始半径
        if (!ok1 || !ok2 || !ok3 || !ok4) {  // 增加ok4检查
            qDebug() << "无效数字";
			message = "Error: 参数异常.";
            return;
        }
    }
    splineColor[0] = 1.0; // 设置为红色 (R=1, G=0, B=0)
    splineColor[1] = 0.0;
    splineColor[2] = 0.0;
    splineLineWidth = 3.0; // 设置曲线线宽
    compute();
}

void MeasureArc::compute() {
    success = false;
    message.clear();
    arcLength = 0.0;
    m_splineActor = nullptr;

    // 重置参数范围
    m_actualParamStart = 0.0;
    m_actualParamEnd = 1.0;
    m_hasVirtualPoints = false;

    if (!m_inputCloud || m_inputCloud->empty()) {
        message = "Error: Input point cloud is empty or invalid.";
        return;
    }
    if (!m_cylinderCoeff || m_cylinderCoeff->values.size() != 7) {
        message = "Error: Cylinder coefficients are invalid (must have 7 values).";
        return;
    }

    // 执行计算流水线
    if (!parseCylinderParameters()) return;
    if (!determineMeasurementPlane(m_specifiedPoint)) return;
    if (!extractAndProjectSlice()) return;
    if (!sortProjectedPoints()) return;
    if (!fitSplineCurve()) return;
    if (!computeArcLengthByIntegration()) return;
    createVisualizationActor();

    success = true;
    message = "Success: Arc length calculated = " + std::to_string(arcLength);
}

bool MeasureArc::parseCylinderParameters() {
    try {
        m_axisPoint = Eigen::Vector3f(m_cylinderCoeff->values[0],
            m_cylinderCoeff->values[1],
            m_cylinderCoeff->values[2]);
        m_axisDirection = Eigen::Vector3f(m_cylinderCoeff->values[3],
            m_cylinderCoeff->values[4],
            m_cylinderCoeff->values[5]).normalized();
        m_cylinderRadius = m_cylinderCoeff->values[6];
        return true;
    }
    catch (...) {
        message = "Error: Failed to parse cylinder coefficients.";
        return false;
    }
}

bool MeasureArc::determineMeasurementPlane(pcl::PointXYZ* specified_point) {
    if (specified_point) {

        Eigen::Vector3f P_specified(specified_point->x,
            specified_point->y,
            specified_point->z);

        // 计算P_specified在轴线上的投影点
        // 参数t = (P_specified - m_axisPoint) · m_axisDirection
        float t = (P_specified - m_axisPoint).dot(m_axisDirection);

        // 投影点 = 轴线起点 + t * 轴线方向
        m_planeCenter = m_axisPoint + t * m_axisDirection;

        qDebug() << "[平面] 使用指定点，已投影到轴线。原点到轴线距离:"
            << (P_specified - m_planeCenter).norm();
    }
    else {
        // 原有代码：自动计算中点
        float t_min = std::numeric_limits<float>::max();
        float t_max = std::numeric_limits<float>::lowest();
        for (const auto& pt : *m_inputCloud) {
            Eigen::Vector3f P(pt.x, pt.y, pt.z);
            float t = (P - m_axisPoint).dot(m_axisDirection);
            t_min = std::min(t_min, t);
            t_max = std::max(t_max, t);
        }
        float t_center = (t_min + t_max) / 2.0f;
        m_planeCenter = m_axisPoint + t_center * m_axisDirection;
        qDebug() << "[平面] 自动计算中点，t=" << t_center;
    }

    // 构建测量平面内的正交基（保持不变）
    Eigen::Vector3f global_x(1.0f, 0.0f, 0.0f);
    if (std::fabs(global_x.dot(m_axisDirection)) > 0.99f) {
        global_x = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }
    m_planeX = global_x - global_x.dot(m_axisDirection) * m_axisDirection;
    m_planeX.normalize();
    m_planeY = m_axisDirection.cross(m_planeX).normalized();

    return true;
}

bool MeasureArc::extractAndProjectSlice() {
    // 计算点云在轴向的平均间距
    float mean_spacing = 0.01f; // 默认值
    if (m_inputCloud->size() > 1) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(m_inputCloud);
        std::vector<int> indices(2);
        std::vector<float> distances(2);
        for (size_t i = 0; i < m_inputCloud->size(); ++i) {
            if (kdtree.nearestKSearch(m_inputCloud->points[i], 2, indices, distances) > 1) {
                mean_spacing += std::sqrt(distances[1]);
            }
        }
        mean_spacing /= m_inputCloud->size();
    }

    float half_thickness = mean_spacing * sliceThicknessFactor / 2.0f;
    m_sliceCloud->clear();
    m_projectedPoints.clear();

    for (const auto& pt : *m_inputCloud) {
        Eigen::Vector3f P(pt.x, pt.y, pt.z);
        float axial_dist = std::abs((P - m_planeCenter).dot(m_axisDirection));
        if (axial_dist <= half_thickness) {
            m_sliceCloud->push_back(pt);
            Eigen::Vector3f v_proj = P - m_planeCenter;
            v_proj = v_proj - v_proj.dot(m_axisDirection) * m_axisDirection;
            float x = v_proj.dot(m_planeX);
            float y = v_proj.dot(m_planeY);
            m_projectedPoints.emplace_back(x, y);
        }
    }

    if (m_projectedPoints.size() < 3) {
        message = "Error: Too few points (" + std::to_string(m_projectedPoints.size()) +
            ") in the slice for curve fitting.";
        return false;
    }

    // 输出关键统计信息
    qDebug() << "[投影统计] 点数:" << m_projectedPoints.size()
        << "，切片厚度:" << half_thickness * 2;
    return true;
}

bool MeasureArc::sortProjectedPoints() {
    if (m_projectedPoints.size() < 3) {
        message = "Error: Too few points for sorting.";
        return false;
    }
    m_sortedPoints.clear();
    m_sortedPoints.reserve(m_projectedPoints.size());

    // 使用圆柱参数圆心(0,0)进行极角排序
    const Eigen::Vector2f circle_center(0.0f, 0.0f);
    std::vector<std::pair<float, size_t>> angle_index_pairs;
    angle_index_pairs.reserve(m_projectedPoints.size());

    for (size_t i = 0; i < m_projectedPoints.size(); ++i) {
        const Eigen::Vector2f& pt = m_projectedPoints[i];
        float raw_angle = std::atan2(pt.y() - circle_center.y(),
            pt.x() - circle_center.x());
        angle_index_pairs.emplace_back(raw_angle, i);
    }

    // 按原始极角升序排序
    std::sort(angle_index_pairs.begin(), angle_index_pairs.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& pair : angle_index_pairs) {
        m_sortedPoints.push_back(m_projectedPoints[pair.second]);
    }

    qDebug() << "[排序] 完成，角度跨度:"
        << (angle_index_pairs.back().first - angle_index_pairs.front().first) * 180.0 / M_PI
        << "度";
    return true;
}

bool MeasureArc::fitSplineCurve() {
    // 数据预处理 - 均匀降采样以获得光滑曲线
    std::vector<Eigen::Vector2f> points_for_fitting;
    int original_size = m_sortedPoints.size();
    int target_size = downsampleTargetSize; // 目标点数，控制平滑度

    if (original_size <= target_size) {
        points_for_fitting = m_sortedPoints;
    }
    else {
        points_for_fitting.reserve(target_size);
        float step = static_cast<float>(original_size - 1) / (target_size - 1);
        for (int i = 0; i < target_size; ++i) {
            int idx = static_cast<int>(std::round(i * step));
            idx = std::min(idx, original_size - 1);
            points_for_fitting.push_back(m_sortedPoints[idx]);
        }
        qDebug() << "[拟合] 降采样:" << original_size << "->" << points_for_fitting.size() << "点";
    }

    if (points_for_fitting.size() < 3) {
        message = "Error: Not enough points after downsampling.";
        return false;
    }

    // 创建CARDINAL_SPLINE对象
    m_vtkSplineX = vtkSmartPointer<vtkCardinalSpline>::New();
    m_vtkSplineY = vtkSmartPointer<vtkCardinalSpline>::New();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    std::vector<Eigen::Vector2f> points_with_virtual = points_for_fitting;

    // 重置虚拟点标志
    m_hasVirtualPoints = false;

    // 添加虚拟端点以稳定样条行为
    if (points_with_virtual.size() >= 2) {
        Eigen::Vector2f head_dir = points_with_virtual[1] - points_with_virtual[0];
        Eigen::Vector2f virtual_head = points_with_virtual[0] - virtualPointExtrapolation * head_dir; // 减少外推距离
        points_with_virtual.insert(points_with_virtual.begin(), virtual_head);

        int last_idx = points_with_virtual.size() - 1;
        Eigen::Vector2f tail_dir = points_with_virtual[last_idx] - points_with_virtual[last_idx - 1];
        Eigen::Vector2f virtual_tail = points_with_virtual[last_idx] + virtualPointExtrapolation * tail_dir; // 减少外推距离
        points_with_virtual.push_back(virtual_tail);

        m_hasVirtualPoints = true;
        qDebug() << "[拟合] 已添加虚拟端点稳定样条行为";
    }

    for (const auto& pt : points_with_virtual) {
        points->InsertNextPoint(pt.x(), pt.y(), 0.0);
    }

    // 弦长参数化
    std::vector<double> cumulative_lengths = { 0.0 };
    for (int i = 1; i < points->GetNumberOfPoints(); ++i) {
        double p0[3], p1[3];
        points->GetPoint(i - 1, p0);
        points->GetPoint(i, p1);
        double dx = p1[0] - p0[0];
        double dy = p1[1] - p0[1];
        double dist = std::sqrt(dx * dx + dy * dy);
        cumulative_lengths.push_back(cumulative_lengths.back() + dist);
    }

    // 归一化参数到 [0, 1]
    double total_length = cumulative_lengths.back();
    if (total_length > 0) {
        for (auto& len : cumulative_lengths) len /= total_length;
    }

    // 计算实际点云的参数范围
    if (m_hasVirtualPoints && cumulative_lengths.size() >= 3) {
        // 第一个点和最后一个点是虚拟点，跳过它们
        m_actualParamStart = cumulative_lengths[1];  // 第二个点（第一个实际点）
        m_actualParamEnd = cumulative_lengths[cumulative_lengths.size() - 2];  // 倒数第二个点（最后一个实际点）
        qDebug() << "[拟合] 实际点云参数范围: [" << m_actualParamStart << ", " << m_actualParamEnd << "]";
    }
    else {
        m_actualParamStart = 0.0;
        m_actualParamEnd = 1.0;
    }

    // 设置样条点
    m_vtkSplineX->RemoveAllPoints();
    m_vtkSplineY->RemoveAllPoints();
    for (size_t i = 0; i < cumulative_lengths.size(); ++i) {
        double p[3];
        points->GetPoint(i, p);
        m_vtkSplineX->AddPoint(cumulative_lengths[i], p[0]);
        m_vtkSplineY->AddPoint(cumulative_lengths[i], p[1]);
    }

    return true;
}

bool MeasureArc::computeArcLengthByIntegration() {
    // 使用中心差分法计算导数的被积函数
    auto integrand = [this](double u) -> double {
        double eps = 1e-6;
        double u_plus = std::min(m_actualParamEnd, u + eps);
        double u_minus = std::max(m_actualParamStart, u - eps);

        double x_plus = m_vtkSplineX->Evaluate(u_plus);
        double y_plus = m_vtkSplineY->Evaluate(u_plus);
        double x_minus = m_vtkSplineX->Evaluate(u_minus);
        double y_minus = m_vtkSplineY->Evaluate(u_minus);

        double dx = (x_plus - x_minus) / (u_plus - u_minus);
        double dy = (y_plus - y_minus) / (u_plus - u_minus);
        return std::sqrt(dx * dx + dy * dy);
        };

    // 自适应辛普森积分
    std::function<double(double, double, double, double, int)> adaptiveSimpson =
        [&](double a, double b, double fa, double fb, int depth) -> double {
        if (depth <= 0) return (b - a) * (fa + fb) / 2.0;

        double c = (a + b) / 2.0;
        double fc = integrand(c);
        double left_est = (c - a) * (fa + fc) / 4.0;
        double right_est = (b - c) * (fc + fb) / 4.0;
        double est = left_est + right_est;

        double finer_est = adaptiveSimpson(a, c, fa, fc, depth - 1) +
            adaptiveSimpson(c, b, fc, fb, depth - 1);

        if (std::fabs(est - finer_est) < integrationTolerance) {
            return finer_est;
        }
        else {
            return adaptiveSimpson(a, c, fa, fc, depth - 1) +
                adaptiveSimpson(c, b, fc, fb, depth - 1);
        }
        };

    try {
        qDebug() << "[积分] 使用参数范围: [" << m_actualParamStart << ", " << m_actualParamEnd << "]";
        double fa = integrand(m_actualParamStart);
        double fb = integrand(m_actualParamEnd);
        arcLength = adaptiveSimpson(m_actualParamStart, m_actualParamEnd, fa, fb, 12);
        qDebug() << "[积分] 完成，实际弧长:" << arcLength;
        return true;
    }
    catch (...) {
        message = "Error: Numerical integration failed.";
        arcLength = 0.0;
        return false;
    }
}

void MeasureArc::createVisualizationActor() {
    const int num_samples = 200; // 增加采样点使曲线更光滑
    vtkSmartPointer<vtkPoints> spline_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    lines->InsertNextCell(num_samples);

    // 在可视化范围内均匀采样
    qDebug() << "[可视化] 采样参数范围: [" << m_actualParamStart << ", " << m_actualParamEnd << "]";

    for (int i = 0; i < num_samples; ++i) {
        // 在实际点云参数范围内均匀采样
        double u = m_actualParamStart + (m_actualParamEnd - m_actualParamStart) *
            (static_cast<double>(i) / (num_samples - 1));
        double x = m_vtkSplineX->Evaluate(u);
        double y = m_vtkSplineY->Evaluate(u);

        // 将2D点转换回3D空间
        Eigen::Vector3f p_3d = m_planeCenter + x * m_planeX + y * m_planeY;
        spline_points->InsertNextPoint(p_3d.x(), p_3d.y(), p_3d.z());
        m_fittedPoints2D.emplace_back(x, y);
        lines->InsertCellPoint(i);

        // 在曲线起点和终点添加标记点
        if (i == 0 || i == num_samples - 1) {
            qDebug() << "[可视化] " << (i == 0 ? "起点" : "终点")
                << "坐标: (" << p_3d.x() << ", " << p_3d.y() << ", " << p_3d.z() << ")";
        }
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(spline_points);
    polyData->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    m_splineActor = vtkSmartPointer<vtkActor>::New();
    m_splineActor->SetMapper(mapper);
    m_splineActor->GetProperty()->SetColor(splineColor[0], splineColor[1], splineColor[2]);
    m_splineActor->GetProperty()->SetLineWidth(splineLineWidth);
    m_splineActor->PickableOff();
}