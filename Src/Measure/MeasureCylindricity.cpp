#include "Measure/MeasureCylindricity.h"
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <random>
#include <cmath>
#include <limits>
#include <iomanip>


MeasureCylindricity::MeasureCylindricity()
    : design_radius_(0.0)
    , tolerance_(2.0)
    , max_iterations_(100)
    , verbose_(true)
    , use_external_initial_line_(false)
    , min_distance_(0.0)
    , max_distance_(0.0)
{
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    inliers_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    outliers_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    heatmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
    // 初始化默认外部直线参数
    external_initial_line_ = LineParams();

    // 初始化最后一次评估结果
    last_assessment_result_ = AssessmentResult();
    last_optimized_line_ = LineParams();
}

MeasureCylindricity::~MeasureCylindricity()
{
}

// 新增：热力图颜色映射函数
void MeasureCylindricity::getColorForDistance(double distance, uint8_t& r, uint8_t& g, uint8_t& b) const
{
    // 将距离映射到0-1的范围
    double normalized_distance = 0.0;
    if (max_distance_ > min_distance_) {
        normalized_distance = (distance - min_distance_) / (max_distance_ - min_distance_);
    }

    // 限制在0-1范围内
    normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));

    // 绿色(0.0) -> 黄色(0.5) -> 红色(1.0)的渐变
    if (normalized_distance < 0.5) {
        // 绿色到黄色：绿色从255递减，红色从0递增
        r = static_cast<uint8_t>(255 * (normalized_distance * 2));
        g = 255;
        b = 0;
    }
    else {
        // 黄色到红色：绿色从255递减，红色保持255
        r = 255;
        g = static_cast<uint8_t>(255 * (2 - normalized_distance * 2));
        b = 0;
    }
}

// 新增：生成热力图点云
void MeasureCylindricity::generateHeatMapCloud(const LineParams& line)
{
    heatmap_cloud_->clear();
    heatmap_cloud_->width = input_cloud_->width;
    heatmap_cloud_->height = input_cloud_->height;
    heatmap_cloud_->is_dense = input_cloud_->is_dense;
    heatmap_cloud_->points.resize(input_cloud_->size());

    // 计算距离范围
    min_distance_ = std::numeric_limits<double>::max();
    max_distance_ = std::numeric_limits<double>::lowest();

    std::vector<double> distances;
    for (const auto& point : input_cloud_->points) {
        if (!isPointValid(point)) continue;

        Eigen::Vector3f p(point.x, point.y, point.z);
        double distance_to_axis = computeDistanceToLine(p, line);
        double deviation = std::abs(distance_to_axis - design_radius_);

        distances.push_back(deviation);
        if (deviation < min_distance_) min_distance_ = deviation;
        if (deviation > max_distance_) max_distance_ = deviation;
    }

    // 如果所有点距离相同，设置一个小的范围避免除零
    if (std::abs(max_distance_ - min_distance_) < 1e-10) {
        max_distance_ = min_distance_ + 1e-5;
    }

    // 为每个点分配颜色
    for (size_t i = 0; i < input_cloud_->points.size(); ++i) {
        const auto& point = input_cloud_->points[i];
        heatmap_cloud_->points[i].x = point.x;
        heatmap_cloud_->points[i].y = point.y;
        heatmap_cloud_->points[i].z = point.z;

        if (isPointValid(point)) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            double distance_to_axis = computeDistanceToLine(p, line);
            double deviation = std::abs(distance_to_axis - design_radius_);

            uint8_t r, g, b;
            getColorForDistance(deviation, r, g, b);

            heatmap_cloud_->points[i].r = r;
            heatmap_cloud_->points[i].g = g;
            heatmap_cloud_->points[i].b = b;
        }
        else {
            // 无效点显示为灰色
            heatmap_cloud_->points[i].r = 128;
            heatmap_cloud_->points[i].g = 128;
            heatmap_cloud_->points[i].b = 128;
        }
    }
}

// ============================================================================
// 新增：获取优化后的轴线系数和参数
// ============================================================================

std::vector<float> MeasureCylindricity::getOptimizedAxisCoeffs() const
{
    return last_optimized_line_.toCoeffs();
}

MeasureCylindricity::LineParams MeasureCylindricity::getOptimizedAxisParams() const
{
    return last_optimized_line_;
}

// ============================================================================
// 原有函数实现（保持不变）
// ============================================================================

void MeasureCylindricity::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud && !cloud->empty()) {
        input_cloud_ = cloud;
        printDebugInfo("点云设置完成，点数: " + std::to_string(input_cloud_->size()));
    }
    else {
        printDebugInfo("错误: 输入点云为空或无效");
    }
}

void MeasureCylindricity::setDesignRadius(double radius)
{
    if (radius > 0) {
        design_radius_ = radius;
        printDebugInfo("设计半径设置为: " + std::to_string(design_radius_));
    }
    else {
        printDebugInfo("错误: 设计半径必须为正数");
    }
}

void MeasureCylindricity::setTolerance(double tolerance)
{
    if (tolerance >= 0) {
        tolerance_ = tolerance;
        printDebugInfo("容差阈值设置为: " + std::to_string(tolerance_));
    }
    else {
        printDebugInfo("错误: 容差必须为非负数");
    }
}

void MeasureCylindricity::setMaxIterations(int max_iters)
{
    if (max_iters > 0) {
        max_iterations_ = max_iters;
        printDebugInfo("最大迭代次数设置为: " + std::to_string(max_iterations_));
    }
    else {
        printDebugInfo("错误: 迭代次数必须为正数");
    }
}

void MeasureCylindricity::setVerbose(bool verbose)
{
    verbose_ = verbose;
}

void MeasureCylindricity::setInitialLine(const Eigen::Vector3f& center, const Eigen::Vector3f& axis)
{
    if (axis.norm() < 1e-6f) {
        printDebugInfo("错误: 方向向量长度为零，使用默认方向");
        external_initial_line_.point = center;
        external_initial_line_.direction = Eigen::Vector3f::UnitZ();
    }
    else {
        external_initial_line_.point = center;
        external_initial_line_.direction = axis.normalized();
    }

    use_external_initial_line_ = true;

    printDebugInfo("设置初始直线参数:");
    printDebugInfo("中心点: (" + std::to_string(center.x()) + ", "
        + std::to_string(center.y()) + ", " + std::to_string(center.z()) + ")");
    printDebugInfo("方向向量: (" + std::to_string(axis.x()) + ", "
        + std::to_string(axis.y()) + ", " + std::to_string(axis.z()) + ")");
}

void MeasureCylindricity::setInitialLine(const LineParams& line)
{
    setInitialLine(line.point, line.direction);
}

void MeasureCylindricity::setInitialLineFromCoeffs(const std::vector<float>& coeffs)
{
    if (coeffs.size() < 6) {
        printDebugInfo("错误: 系数数组长度不足6，需要[center_x, center_y, center_z, axis_x, axis_y, axis_z]");
        return;
    }

    Eigen::Vector3f center(coeffs[0], coeffs[1], coeffs[2]);
    Eigen::Vector3f axis(coeffs[3], coeffs[4], coeffs[5]);

    setInitialLine(center, axis);
}

bool MeasureCylindricity::isPointValid(const pcl::PointXYZ& point) const
{
    return pcl::isFinite(point) &&
        !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z);
}

double MeasureCylindricity::computeDistanceToLine(const Eigen::Vector3f& point,
    const LineParams& line)
{
    Eigen::Vector3f v = point - line.point;
    Eigen::Vector3f cross_product = v.cross(line.direction);
    return cross_product.norm();
}

double MeasureCylindricity::objectiveFunction(const LineParams& line)
{
    if (!input_cloud_ || input_cloud_->empty()) {
        return std::numeric_limits<double>::max();
    }

    double total_squared_error = 0.0;
    int valid_points = 0;

    for (const auto& p : input_cloud_->points) {
        if (!isPointValid(p)) continue;

        Eigen::Vector3f point(p.x, p.y, p.z);
        double distance = computeDistanceToLine(point, line);
        double error = distance - design_radius_;  // 差值
        total_squared_error += error * error;      // 平方和
        valid_points++;
    }

    return valid_points > 0 ? total_squared_error / valid_points : std::numeric_limits<double>::max();
}

MeasureCylindricity::LineParams MeasureCylindricity::initializeLineParameters()
{
    if (use_external_initial_line_) {
        printDebugInfo("使用外部提供的初始直线参数");
        return external_initial_line_;
    }

    LineParams initial_line;

    if (!isValidPointCloud()) {
        printDebugInfo("警告: 点云无效，使用默认直线参数");
        initial_line.point = Eigen::Vector3f::Zero();
        initial_line.direction = Eigen::Vector3f::UnitZ();
        return initial_line;
    }

    try {
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(input_cloud_);

        Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
        Eigen::Vector3f eigenvalues = pca.getEigenValues();

        int max_index = 0;
        for (int i = 1; i < 3; ++i) {
            if (eigenvalues(i) > eigenvalues(max_index)) {
                max_index = i;
            }
        }

        initial_line.direction = eigenvectors.col(max_index);
        initial_line.direction.normalize();
        initial_line.point = computeCentroid();

        printDebugInfo("使用PCA初始化直线参数");
        printDebugInfo("初始方向: (" + std::to_string(initial_line.direction.x()) +
            ", " + std::to_string(initial_line.direction.y()) +
            ", " + std::to_string(initial_line.direction.z()) + ")");

    }
    catch (const std::exception& e) {
        printDebugInfo("PCA初始化失败: " + std::string(e.what()));
        initial_line.point = computeCentroid();
        initial_line.direction = Eigen::Vector3f::UnitZ();
    }

    return initial_line;
}

MeasureCylindricity::LineParams MeasureCylindricity::perturbLineParameters(
    const LineParams& line, float magnitude)
{
    LineParams perturbed_line = line;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0f, magnitude);

    Eigen::Vector3f tangent1 = line.direction.unitOrthogonal();
    Eigen::Vector3f tangent2 = line.direction.cross(tangent1);

    float angle1 = dist(gen) * 0.1f;
    float angle2 = dist(gen) * 0.1f;

    Eigen::AngleAxisd rotation1(angle1, tangent1.cast<double>());
    Eigen::AngleAxisd rotation2(angle2, tangent2.cast<double>());
    perturbed_line.direction = (rotation2 * rotation1 * line.direction.cast<double>()).cast<float>();
    perturbed_line.direction.normalize();

    perturbed_line.point += tangent1 * dist(gen) * 0.01f;
    perturbed_line.point += tangent2 * dist(gen) * 0.01f;

    return perturbed_line;
}

MeasureCylindricity::LineParams MeasureCylindricity::levenbergMarquardtOptimization()
{
    LineParams current_line = initializeLineParameters();
    double current_error = objectiveFunction(current_line);
    double best_error = current_error;
    LineParams best_line = current_line;

    double lambda = 0.001;
    int max_inner_iterations = 10;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<LineParams> perturbations;
        std::vector<double> errors;

        for (int i = 0; i < 8; ++i) {
            LineParams perturbed = perturbLineParameters(current_line, 0.01f);
            double error = objectiveFunction(perturbed);
            perturbations.push_back(perturbed);
            errors.push_back(error);
        }

        int best_idx = -1;
        for (size_t i = 0; i < errors.size(); ++i) {
            if (errors[i] < current_error) {
                current_error = errors[i];
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            current_line = perturbations[best_idx];
            if (current_error < best_error) {
                best_error = current_error;
                best_line = current_line;
            }
        }
        else {
            lambda *= 2.0;
        }

        if (iter % 20 == 0 && verbose_) {
            printDebugInfo("LM优化迭代 " + std::to_string(iter) +
                ", 误差: " + std::to_string(best_error));
        }

        if (iter > 10 && best_error < tolerance_ * 0.1) {
            break;
        }
    }

    return best_line;
}

MeasureCylindricity::LineParams MeasureCylindricity::randomSearchOptimization()
{
    LineParams best_line = initializeLineParameters();
    double best_error = objectiveFunction(best_line);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> angle_dist(-1.0f, 1.0f);
    std::normal_distribution<float> pos_dist(0.0f, 0.1f);

    int search_iterations = std::min(max_iterations_, 200);

    for (int i = 0; i < search_iterations; ++i) {
        LineParams test_line = best_line;

        Eigen::Vector3f random_vec(angle_dist(gen), angle_dist(gen), angle_dist(gen));
        random_vec.normalize();

        float mix_factor = 0.3f;
        test_line.direction = (best_line.direction * (1.0f - mix_factor) +
            random_vec * mix_factor).normalized();

        test_line.point = best_line.point +
            Eigen::Vector3f(pos_dist(gen), pos_dist(gen), pos_dist(gen)) * 0.01f;

        double error = objectiveFunction(test_line);

        if (error < best_error) {
            best_error = error;
            best_line = test_line;
        }

        if (i % 40 == 0 && verbose_) {
            printDebugInfo("随机搜索迭代 " + std::to_string(i) +
                ", 最佳误差: " + std::to_string(best_error));
        }
    }

    return best_line;
}

MeasureCylindricity::LineParams MeasureCylindricity::optimizeLineParameters()
{
    printDebugInfo("开始优化直线参数（4自由度）...");

    if (use_external_initial_line_) {
        printDebugInfo("使用外部提供的初始直线作为优化起点");
    }

    LineParams result1 = randomSearchOptimization();
    double error1 = objectiveFunction(result1);

    LineParams result2 = levenbergMarquardtOptimization();
    double error2 = objectiveFunction(result2);

    if (error1 < error2) {
        printDebugInfo("选择随机搜索结果，误差: " + std::to_string(error1));
        return result1;
    }
    else {
        printDebugInfo("选择LM优化结果，误差: " + std::to_string(error2));
        return result2;
    }
}

MeasureCylindricity::AssessmentResult MeasureCylindricity::computeAssessmentMetrics(
    const LineParams& line)
{
    AssessmentResult result;
    distance_map_.clear();
    inliers_->clear();
    outliers_->clear();

    result.design_radius = design_radius_;

    if (!input_cloud_ || input_cloud_->empty()) {
        result.assessment_message = "错误: 点云为空";
        return result;
    }

    double sum_distance = 0.0;
    double sum_squared_distance = 0.0;
    double sum_squared_deviation = 0.0;
    int outlier_count = 0;
    int valid_points = 0;

    std::vector<double> deviations;

    // 新增：初始化最小偏差为最大值
    result.min_deviation = std::numeric_limits<double>::max();
    result.max_deviation = 0.0;


    for (const auto& p : input_cloud_->points) {
        if (!isPointValid(p)) continue;

        Eigen::Vector3f point(p.x, p.y, p.z);
        double distance_to_axis = computeDistanceToLine(point, line);
        double deviation = std::abs(distance_to_axis - design_radius_);

        distance_map_.push_back(deviation);
        deviations.push_back(deviation);
        sum_distance += deviation;
        sum_squared_distance += deviation * deviation;
        valid_points++;

        if (deviation > tolerance_) {
            outlier_count++;
            outliers_->push_back(p);
        }
        else {
            inliers_->push_back(p);
        }
        // 更新最小偏差
        if (deviation < result.min_deviation) {
            result.min_deviation = deviation;
        }

        if (deviation > result.max_deviation) {
            result.max_deviation = deviation;
        }
    }

    if (valid_points > 0) {
        result.outlier_ratio = static_cast<double>(outlier_count) / valid_points;
        result.mean_deviation = sum_distance / valid_points;
        result.rms_error = std::sqrt(sum_squared_distance / valid_points);
        for (double dev : deviations) {
            sum_squared_deviation += std::pow(dev - result.mean_deviation, 2);
        }
        result.uniformity_rms = std::sqrt(sum_squared_deviation / valid_points);
        result.is_acceptable = (result.outlier_ratio < 0.1);
        result.best_fit_line = line;
    }
    else {
        result.min_deviation = 0.0;  // 新增：无有效点时的默认值
        result.outlier_ratio = 1.0;
        result.mean_deviation = tolerance_ * 2;
        result.rms_error = tolerance_ * 2;
        result.uniformity_rms = tolerance_ * 2;
        result.is_acceptable = false;
    }

    return result;
}

MeasureCylindricity::AssessmentResult MeasureCylindricity::evaluateGivenLine(
    const Eigen::Vector3f& center, const Eigen::Vector3f& axis)
{
    AssessmentResult result;

    if (!isValidPointCloud()) {
        result.assessment_message = "错误: 输入点云无效";
        result.is_acceptable = false;
        return result;
    }

    if (design_radius_ <= 0) {
        result.assessment_message = "错误: 设计半径必须为正数";
        result.is_acceptable = false;
        return result;
    }

    printDebugInfo("开始直接评估给定直线的圆柱度...");
    printDebugInfo("点云点数: " + std::to_string(input_cloud_->size()));
    printDebugInfo("设计半径: " + std::to_string(design_radius_));
    printDebugInfo("容差阈值: " + std::to_string(tolerance_));

    try {
        LineParams given_line(center, axis);
        result = computeAssessmentMetrics(given_line);

        std::stringstream message;
        message << "=== 圆柱度直接评估报告 ===\n";
        message << "点云点数: " << input_cloud_->size() << "\n";
        message << "设计半径: " << std::fixed << std::setprecision(4) << design_radius_ << " mm\n";
        message << "容差阈值: " << tolerance_ << " mm\n\n";

        message << "给定直线参数:\n";
        message << "轴线点: (" << std::setprecision(3)
            << center.x() << ", " << center.y() << ", " << center.z() << ")\n";
        message << "轴线方向: (" << axis.x() << ", " << axis.y() << ", " << axis.z() << ")\n\n";

        message << "评估结果:\n";
        message << "不贴合点比例: " << std::setprecision(2) << result.outlier_ratio * 100 << "%\n";
        message << "最小偏差: " << std::setprecision(4) << result.min_deviation << " mm\n";
        message << "最大偏差: " << std::setprecision(4) << result.max_deviation << " mm\n";
        message << "平均偏差: " << result.mean_deviation << " m\n";
        message << "RMS偏差: " << result.rms_error << " m\n\n";
        message << "均匀性(标准差): " << result.uniformity_rms << " mm\n\n";

        message << (result.is_acceptable ? "✅ 圆柱度良好" : "❌ 圆柱度不佳");
        message << "（不贴合点比例" << (result.is_acceptable ? "＜" : "≥") << "10%）";

        result.assessment_message = message.str();

        // 保存最后一次评估结果（但不保存为优化结果）
        last_assessment_result_ = result;

    }
    catch (const std::exception& e) {
        result.assessment_message = "评估过程出错: " + std::string(e.what());
        result.is_acceptable = false;
    }

    printDebugInfo(result.assessment_message);
    return result;
}

MeasureCylindricity::AssessmentResult MeasureCylindricity::evaluateCylindricity()
{
    AssessmentResult result;

    if (!isValidPointCloud()) {
        result.assessment_message = "错误: 输入点云无效";
        result.is_acceptable = false;
        return result;
    }

    if (design_radius_ <= 0) {
        result.assessment_message = "错误: 设计半径必须为正数";
        result.is_acceptable = false;
        return result;
    }

    printDebugInfo("开始圆柱度评估（新思路）...");
    printDebugInfo("点云点数: " + std::to_string(input_cloud_->size()));
    printDebugInfo("设计半径: " + std::to_string(design_radius_));
    printDebugInfo("容差阈值: " + std::to_string(tolerance_));

    try {
        LineParams best_line = optimizeLineParameters();
        result = computeAssessmentMetrics(best_line);

        // 新增：生成热力图点云
        generateHeatMapCloud(best_line);

        std::stringstream message;
        message << "=== 圆柱度评估报告===\n";
        message << "点云点数: " << input_cloud_->size() << "\n";
        message << "设计半径: " << std::fixed << std::setprecision(4) << design_radius_ << " mm\n";
        message << "容差阈值: " << tolerance_ << " mm\n\n";

        message << "拟合结果:\n";
        message << "不贴合点比例: " << std::setprecision(2) << result.outlier_ratio * 100 << "%\n";
        message << "最小偏差: " << std::setprecision(4) << result.min_deviation << " mm\n";
        message << "最大偏差: " << std::setprecision(4) << result.max_deviation << " mm\n";
        message << "平均偏差: " << result.mean_deviation << " mm\n";
        message << "RMS误差: " << result.rms_error << " mm\n\n";
        message << "均匀性(标准差): " << result.uniformity_rms << " mm\n\n";

        message << "圆柱轴线参数:\n";
        message << "轴线点: (" << std::setprecision(3)
            << best_line.point.x() << ", " << best_line.point.y() << ", "
            << best_line.point.z() << ")\n";
        message << "轴线方向: (" << best_line.direction.x() << ", "
            << best_line.direction.y() << ", " << best_line.direction.z() << ")\n\n";

        message << (result.is_acceptable ? "✅ 圆柱度良好" : "❌ 圆柱度不佳");
        message << "（不贴合点比例" << (result.is_acceptable ? "＜" : "≥") << "10%）";

        result.assessment_message = message.str();

        // 保存最后一次评估结果和优化后的轴线
        last_assessment_result_ = result;
        last_optimized_line_ = best_line;

    }
    catch (const std::exception& e) {
        result.assessment_message = "评估过程出错: " + std::string(e.what());
        result.is_acceptable = false;
    }

    printDebugInfo(result.assessment_message);
    return result;
}

Eigen::Vector3f MeasureCylindricity::computeCentroid() const
{
    if (!input_cloud_ || input_cloud_->empty()) {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud_, centroid);
    return centroid.head<3>();
}

bool MeasureCylindricity::isValidPointCloud() const
{
    return input_cloud_ && !input_cloud_->empty();
}

void MeasureCylindricity::printDebugInfo(const std::string& message) const
{
    if (verbose_) {
        std::cout << "[MeasureCylindricity] " << message << std::endl;
    }
}
