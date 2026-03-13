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

// 添加Eigen密集矩阵运算头文件
#include <Eigen/Dense>

MeasureArc::MeasureArc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::ModelCoefficients::Ptr cylinder_coeff,
    pcl::PointXYZ* specified_point,
    FitMethod method)
    : m_inputCloud(cloud)
    , m_cylinderCoeff(cylinder_coeff)
    , m_specifiedPoint(specified_point)
    , m_sliceCloud(new pcl::PointCloud<pcl::PointXYZ>)
    , fitMethod(method)
    , arcLength(0.0)
    , success(false)
    , message("Initialized.")
    , m_actualParamStart(0.0)
    , m_actualParamEnd(1.0)
    , m_hasVirtualPoints(false)
    , m_bsplineParamStart(0.0)
    , m_bsplineParamEnd(1.0)
{
    // 初始化成员变量
    m_axisPoint = Eigen::Vector3f::Zero();
    m_axisDirection = Eigen::Vector3f::Zero();
    m_cylinderRadius = 0.0;
    m_planeCenter = Eigen::Vector3f::Zero();
    m_planeX = Eigen::Vector3f::Zero();
    m_planeY = Eigen::Vector3f::Zero();

    if (method == CARDINAL_SPLINE) {
        ParamDialogMeaArc dialog;
        if (dialog.exec() != QDialog::Accepted) {
			isCancelled = true;
			message = ">>:操作取消";
            return;
        }

        bool ok1, ok2, ok3, ok4;
        QString param1 = dialog.getParams()[0];
        QString param2 = dialog.getParams()[1];
        QString param3 = dialog.getParams()[2];
        QString param4 = dialog.getParams()[3];
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
    else {
        ParamDialogMeaArcB dialog;
        if (dialog.exec() != QDialog::Accepted) {
            isCancelled = true;
            message = ">>:操作取消";
            return;
        }

        bool ok1, ok2, ok3, ok4, ok5;
        QString param1 = dialog.getParams()[0];
        QString param2 = dialog.getParams()[1];
        QString param3 = dialog.getParams()[2];
        QString param4 = dialog.getParams()[3];
        QString param5 = dialog.getParams()[4];
        sliceThicknessFactor = param1.toDouble(&ok1);
        integrationTolerance = param2.toDouble(&ok2);
        bsplineDegree = param3.toInt(&ok3);
        bsplineControlPoints = param4.toInt(&ok4);
		bsplineSmoothingFactor = param5.toDouble(&ok5);
        if (!ok1 || !ok2 || !ok3 || !ok4 || !ok5){  // 增加ok4检查
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

bool MeasureArc::fitSplineCurve() {
    qDebug() << "[拟合] 使用拟合方法:"
        << (fitMethod == CARDINAL_SPLINE ? "CARDINAL_SPLINE" : "BSPLINE_LSQ");

    if (fitMethod == CARDINAL_SPLINE) {
        return fitCardinalSpline();
    }
    else {
        return fitBSplineLeastSquares();
    }
}

bool MeasureArc::fitCardinalSpline() {
    // 数据预处理 - 均匀降采样以获得光滑曲线
    std::vector<Eigen::Vector2f> points_for_fitting;
    int original_size = m_sortedPoints.size();
    int target_size = downsampleTargetSize;

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
        qDebug() << "[Cardinal拟合] 降采样:" << original_size << "->" << points_for_fitting.size() << "点";
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
        Eigen::Vector2f virtual_head = points_with_virtual[0] - virtualPointExtrapolation * head_dir;
        points_with_virtual.insert(points_with_virtual.begin(), virtual_head);

        int last_idx = points_with_virtual.size() - 1;
        Eigen::Vector2f tail_dir = points_with_virtual[last_idx] - points_with_virtual[last_idx - 1];
        Eigen::Vector2f virtual_tail = points_with_virtual[last_idx] + virtualPointExtrapolation * tail_dir;
        points_with_virtual.push_back(virtual_tail);

        m_hasVirtualPoints = true;
        qDebug() << "[Cardinal拟合] 已添加虚拟端点 (外推比例:" << virtualPointExtrapolation << ")";
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
        m_actualParamStart = cumulative_lengths[1];
        m_actualParamEnd = cumulative_lengths[cumulative_lengths.size() - 2];
        qDebug() << "[Cardinal拟合] 实际点云参数范围: [" << m_actualParamStart << ", " << m_actualParamEnd << "]";
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

// B样条基函数计算
double MeasureArc::bsplineBasis(int i, int k, double t, const std::vector<double>& knots) {
    if (k == 0) {
        // 对于零次基函数
        if (t >= knots[i] && t < knots[i + 1]) {
            return 1.0;
        }
        else if (t == knots.back() && i == m_bsplineCoeffsX.size() - 1) {
            // 处理最后一个节点：当t等于最后一个节点时，只有最后一个基函数返回1
            return 1.0;
        }
        else {
            return 0.0;
        }
    }

    double denom1 = knots[i + k] - knots[i];
    double denom2 = knots[i + k + 1] - knots[i + 1];

    double term1 = 0.0, term2 = 0.0;

    if (denom1 != 0.0) {
        term1 = ((t - knots[i]) / denom1) * bsplineBasis(i, k - 1, t, knots);
    }

    if (denom2 != 0.0) {
        term2 = ((knots[i + k + 1] - t) / denom2) * bsplineBasis(i + 1, k - 1, t, knots);
    }

    return term1 + term2;
}

// 生成均匀节点向量
std::vector<double> MeasureArc::generateUniformKnots(int n, int p) {
    std::vector<double> knots(n + p + 1);
    int m = n + p;

    // 前p+1个节点为0
    for (int i = 0; i <= p; ++i) {
        knots[i] = 0.0;
    }

    // 内部节点均匀分布
    if (n > p + 1) {
        for (int i = p + 1; i < n; ++i) {
            knots[i] = static_cast<double>(i - p) / (n - p);
        }
    }
    else {
        // 如果控制点太少，内部节点均匀分布
        for (int i = p + 1; i < n; ++i) {
            knots[i] = 0.5; // 中间位置
        }
    }

    // 后p+1个节点为1
    for (int i = n; i <= m; ++i) {
        knots[i] = 1.0;
    }

    return knots;
}

// B样条曲线求值
double MeasureArc::evaluateBSpline(double t, const std::vector<double>& coeffsX,
    const std::vector<double>& coeffsY,
    const std::vector<double>& knots) {
    int n = coeffsX.size();  // 控制点数量
    int p = bsplineDegree;

    // 确保t在有效范围内
    t = std::max(knots[p], std::min(t, knots[n]));

    double resultX = 0.0;
    for (int i = 0; i < n; ++i) {
        double basis = bsplineBasis(i, p, t, knots);
        resultX += coeffsX[i] * basis;
    }

    return resultX;
}

// B样条曲线导数求值
std::pair<double, double> MeasureArc::evaluateBSplineDerivative(double t,
    const std::vector<double>& coeffsX,
    const std::vector<double>& coeffsY,
    const std::vector<double>& knots) {
    int n = coeffsX.size();  // 控制点数量
    int p = bsplineDegree;

    // 确保t在有效范围内
    t = std::max(knots[p], std::min(t, knots[n]));

    double dx = 0.0, dy = 0.0;

    // 计算一阶导数（使用B样条导数公式）
    for (int i = 0; i < n - 1; ++i) {
        double denom = knots[i + p + 1] - knots[i + 1];
        if (denom > 0.0) {
            double basis = bsplineBasis(i + 1, p - 1, t, knots);
            dx += p * (coeffsX[i + 1] - coeffsX[i]) / (knots[i + p + 1] - knots[i + 1]) * basis;
            dy += p * (coeffsY[i + 1] - coeffsY[i]) / (knots[i + p + 1] - knots[i + 1]) * basis;
        }
    }

    return { dx, dy };
}

bool MeasureArc::fitBSplineLeastSquares() {
    int n = m_sortedPoints.size();
    if (n < 3) {
        message = "Error: Not enough points for B-spline fitting.";
        return false;
    }

    // 检查控制点数量是否合理
    if (bsplineControlPoints >= n) {
        bsplineControlPoints = n - 1;  // 修正：控制点数量必须小于数据点数量
        qDebug() << "[B样条] 控制点数量调整:" << bsplineControlPoints;
    }

    int p = bsplineDegree;  // B样条次数
    int m = bsplineControlPoints;  // 控制点数量

    // 调试输出
    qDebug() << "[B样条逼近] 数据点数:" << n
        << ", 控制点数:" << m
        << ", 样条次数:" << p
        << ", 平滑因子:" << bsplineSmoothingFactor;

    // 1. 弦长参数化
    std::vector<double> params(n);
    params[0] = 0.0;

    for (int i = 1; i < n; ++i) {
        double dx = m_sortedPoints[i].x() - m_sortedPoints[i - 1].x();
        double dy = m_sortedPoints[i].y() - m_sortedPoints[i - 1].y();
        params[i] = params[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    // 归一化到 [0, 1]
    if (params[n - 1] > 0.0) {
        for (int i = 0; i < n; ++i) {
            params[i] /= params[n - 1];
        }
    }
    else {
        // 如果所有点重合，使用均匀参数化
        for (int i = 0; i < n; ++i) {
            params[i] = static_cast<double>(i) / (n - 1);
        }
    }

    // 2. 生成节点向量
    m_bsplineKnots = generateUniformKnots(m, p);

    // 3. 构建设计矩阵 B (n × m)
    Eigen::MatrixXd B(n, m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            B(i, j) = bsplineBasis(j, p, params[i], m_bsplineKnots);
        }
    }

    // 4. 构建数据向量
    Eigen::VectorXd Qx(n), Qy(n);
    for (int i = 0; i < n; ++i) {
        Qx(i) = m_sortedPoints[i].x();
        Qy(i) = m_sortedPoints[i].y();
    }

    // 5. 添加平滑项（正则化）
    Eigen::MatrixXd D(m, m);
    D.setZero();

    // 二阶差分矩阵（鼓励曲线平滑）
    for (int i = 2; i < m; ++i) {
        D(i, i - 2) = 1.0;
        D(i, i - 1) = -2.0;
        D(i, i) = 1.0;
    }

    // 6. 解最小二乘问题：(B^T B + λ D^T D) P = B^T Q
    Eigen::MatrixXd A = B.transpose() * B + bsplineSmoothingFactor * D.transpose() * D;
    Eigen::VectorXd bx = B.transpose() * Qx;
    Eigen::VectorXd by = B.transpose() * Qy;

    // 7. 解线性方程组
    Eigen::VectorXd Px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd Py = A.colPivHouseholderQr().solve(by);

    // 8. 存储控制点系数
    m_bsplineCoeffsX.resize(m);
    m_bsplineCoeffsY.resize(m);
    for (int i = 0; i < m; ++i) {
        m_bsplineCoeffsX[i] = Px(i);
        m_bsplineCoeffsY[i] = Py(i);

        // 调试输出：检查控制点是否合理
        qDebug() << "[控制点" << i << "] X=" << Px(i) << ", Y=" << Py(i);
    }

    // 9. 设置参数范围
    m_bsplineParamStart = m_bsplineKnots[p];
    m_bsplineParamEnd = m_bsplineKnots[m];
    m_actualParamStart = 0.0;
    m_actualParamEnd = 1.0;

    qDebug() << "[B样条逼近] 拟合完成，参数范围: ["
        << m_bsplineParamStart << ", " << m_bsplineParamEnd << "]";

    return true;
}

bool MeasureArc::computeArcLengthByIntegration() {
    if (fitMethod == CARDINAL_SPLINE) {
        // 使用中心差分法计算导数的被积函数（Cardinal样条）
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

        qDebug() << "[Cardinal积分] 使用参数范围: [" << m_actualParamStart << ", " << m_actualParamEnd << "]";

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
            double fa = integrand(m_actualParamStart);
            double fb = integrand(m_actualParamEnd);
            arcLength = adaptiveSimpson(m_actualParamStart, m_actualParamEnd, fa, fb, 12);
            qDebug() << "[Cardinal积分] 完成，弧长:" << arcLength;
            return true;
        }
        catch (...) {
            message = "Error: Numerical integration failed.";
            arcLength = 0.0;
            return false;
        }
    }
    else {
        // B样条逼近的弧长计算
        auto integrand = [this](double t) -> double {
            auto deriv = evaluateBSplineDerivative(t, m_bsplineCoeffsX, m_bsplineCoeffsY, m_bsplineKnots);
            return std::sqrt(deriv.first * deriv.first + deriv.second * deriv.second);
            };

        qDebug() << "[B样条积分] 使用参数范围: [" << m_bsplineParamStart << ", " << m_bsplineParamEnd << "]";

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
            double fa = integrand(m_bsplineParamStart);
            double fb = integrand(m_bsplineParamEnd);
            arcLength = adaptiveSimpson(m_bsplineParamStart, m_bsplineParamEnd, fa, fb, 12);
            qDebug() << "[B样条积分] 完成，弧长:" << arcLength;
            return true;
        }
        catch (...) {
            message = "Error: Numerical integration failed.";
            arcLength = 0.0;
            return false;
        }
    }
}

void MeasureArc::createVisualizationActor() {
    const int num_samples = 200;
    vtkSmartPointer<vtkPoints> spline_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    lines->InsertNextCell(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double x, y;

        if (fitMethod == CARDINAL_SPLINE) {
            // Cardinal样条保持不变
            double u = m_actualParamStart + (m_actualParamEnd - m_actualParamStart) * t;
            x = m_vtkSplineX->Evaluate(u);
            y = m_vtkSplineY->Evaluate(u);
        }
        else {
            // B样条逼近 - 修复参数映射
            int n = m_bsplineCoeffsX.size();
            int p = bsplineDegree;

            // 确保采样参数在有效范围内
            double u = m_bsplineParamStart + (m_bsplineParamEnd - m_bsplineParamStart) * t;

            // 避免浮点误差导致超出有效范围
            u = std::max(m_bsplineKnots[p], std::min(u, m_bsplineKnots[n]));

            // 对于最后一个采样点，确保处理右端点
            if (i == num_samples - 1) {
                u = m_bsplineKnots[n]; // 使用最后一个节点
            }

            // 同时计算X和Y坐标
            x = 0.0;
            y = 0.0;
            for (int j = 0; j < n; ++j) {
                double basis = bsplineBasis(j, p, u, m_bsplineKnots);
                x += m_bsplineCoeffsX[j] * basis;
                y += m_bsplineCoeffsY[j] * basis;
            }

            // 调试输出：查看前几个和后几个点的坐标
            if (i < 5 || i > num_samples - 5) {
                qDebug() << "[B样条可视化] 点" << i << ": t=" << t << ", u=" << u
                    << ", x=" << x << ", y=" << y;
            }
        }

        // 将2D点转换回3D空间
        Eigen::Vector3f p_3d = m_planeCenter + x * m_planeX + y * m_planeY;
        spline_points->InsertNextPoint(p_3d.x(), p_3d.y(), p_3d.z());
        m_fittedPoints2D.emplace_back(x, y);
        lines->InsertCellPoint(i);
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

