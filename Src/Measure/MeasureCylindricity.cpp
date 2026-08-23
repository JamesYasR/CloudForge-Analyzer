#include "Measure/MeasureCylindricity.h"
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <random>
#include <cmath>
#include <limits>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <functional>
#include <omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

MeasureCylindricity::MeasureCylindricity()
    : design_radius_(0.0)
    , tolerance_(2.0)
    , max_iterations_(100)
    , verbose_(true)
    , use_external_initial_line_(false)
    , min_distance_(0.0)
    , max_distance_(0.0)
    , weld_constraint_applied_(false)
    , weld_thr_factor_(3.0)
    , weld_cluster_tolerance_(2.0)
    , weld_min_cluster_size_(50)
    , weld_lambda_(1.0)
{
    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    inliers_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    outliers_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    heatmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
    weld_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);
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

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(input_cloud_->size()); ++i) {
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
            // 无效点处理
            heatmap_cloud_->points[i].r = 128;
            heatmap_cloud_->points[i].g = 128;
            heatmap_cloud_->points[i].b = 128;
        }
    }
}


std::vector<float> MeasureCylindricity::getOptimizedAxisCoeffs() const
{
    return last_optimized_line_.toCoeffs();
}

MeasureCylindricity::LineParams MeasureCylindricity::getOptimizedAxisParams() const
{
    return last_optimized_line_;
}

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

// 新增：焊缝检测参数设置（第三阶段）
void MeasureCylindricity::setWeldThresholdFactor(double k)
{
    if (k > 0) {
        weld_thr_factor_ = k;
        printDebugInfo("焊缝阈值系数设置为: " + std::to_string(k));
    }
    else {
        printDebugInfo("错误: 焊缝阈值系数必须为正数");
    }
}

void MeasureCylindricity::setWeldClusterTolerance(double eps)
{
    if (eps > 0) {
        weld_cluster_tolerance_ = eps;
        printDebugInfo("焊缝聚类容差设置为: " + std::to_string(eps) + " mm");
    }
    else {
        printDebugInfo("错误: 焊缝聚类容差必须为正数");
    }
}

void MeasureCylindricity::setWeldMinClusterSize(int n)
{
    if (n > 0) {
        weld_min_cluster_size_ = n;
        printDebugInfo("焊缝最小簇点数设置为: " + std::to_string(n));
    }
    else {
        printDebugInfo("错误: 焊缝最小簇点数必须为正数");
    }
}

void MeasureCylindricity::setWeldConstraintWeight(double lambda)
{
    if (lambda >= 0) {
        weld_lambda_ = lambda;
        printDebugInfo("焊缝约束权重设置为: " + std::to_string(lambda));
    }
    else {
        printDebugInfo("错误: 焊缝约束权重必须非负");
    }
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

double MeasureCylindricity::objectiveFunction(const LineParams& line) {
    if (!input_cloud_ || input_cloud_->empty()) {
        return std::numeric_limits<double>::max();
    }

    double total_squared_error = 0.0;
    int valid_points = 0;

    //并行化
#pragma omp parallel for reduction(+:total_squared_error, valid_points)
    for (int i = 0; i < static_cast<int>(input_cloud_->size()); ++i) {
        const auto& p = input_cloud_->points[i];
        if (!isPointValid(p)) continue;

        Eigen::Vector3f point(p.x, p.y, p.z);
        double distance = computeDistanceToLine(point, line);
        double error = distance - design_radius_;
        total_squared_error += error * error;
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

//MeasureCylindricity::LineParams MeasureCylindricity::levenbergMarquardtOptimization()
//{
//    LineParams current_line = initializeLineParameters();
//    double current_error = objectiveFunction(current_line);
//    double best_error = current_error;
//    LineParams best_line = current_line;
//
//    double lambda = 0.001;
//    int max_inner_iterations = 10;
//
//    for (int iter = 0; iter < max_iterations_; ++iter) {
//        std::vector<LineParams> perturbations;
//        std::vector<double> errors;
//
//        for (int i = 0; i < 8; ++i) {
//            LineParams perturbed = perturbLineParameters(current_line, 0.01f);
//            double error = objectiveFunction(perturbed);
//            perturbations.push_back(perturbed);
//            errors.push_back(error);
//        }
//
//        int best_idx = -1;
//        for (size_t i = 0; i < errors.size(); ++i) {
//            if (errors[i] < current_error) {
//                current_error = errors[i];
//                best_idx = i;
//            }
//        }
//
//        if (best_idx >= 0) {
//            current_line = perturbations[best_idx];
//            if (current_error < best_error) {
//                best_error = current_error;
//                best_line = current_line;
//            }
//        }
//        else {
//            lambda *= 2.0;
//        }
//
//        if (iter % 20 == 0 && verbose_) {
//            printDebugInfo("LM优化迭代 " + std::to_string(iter) +
//                ", 误差: " + std::to_string(best_error));
//        }
//
//        if (iter > 10 && best_error < tolerance_ * 0.1) {
//            break;
//        }
//    }
//
//    return best_line;
//}
MeasureCylindricity::LineParams MeasureCylindricity::levenbergMarquardtOptimization()
{
    LineParams current_line = initializeLineParameters();
    double current_error = objectiveFunction(current_line);
    double best_error = current_error;
    LineParams best_line = current_line;

    // 动态步长参数
    std::vector<float> step_sizes = { 0.1f, 0.05f, 0.01f, 0.005f };  // 多尺度步长
    int current_step_idx = 0;

    int stuck_counter = 0;
    const int max_stuck_iterations = 5;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        float current_step = step_sizes[current_step_idx];

        // 生成更多样化的扰动
        std::vector<LineParams> perturbations;
        std::vector<double> errors;

        // 生成不同类型的扰动
        for (int i = 0; i < 12; ++i) {  // 增加扰动数量
            float perturbation_factor = 1.0f;

            if (i < 4) {  // 小步长精确搜索
                perturbation_factor = 0.5f;
            }
            else if (i < 8) {  // 中步长探索
                perturbation_factor = 1.0f;
            }
            else {  // 大步长探索
                perturbation_factor = 2.0f;
            }

            LineParams perturbed = perturbLineParameters(current_line,
                current_step * perturbation_factor);
            double error = objectiveFunction(perturbed);
            perturbations.push_back(perturbed);
            errors.push_back(error);
        }

        // 添加一个随机重启
        if (iter % 10 == 0 && stuck_counter >= 2) {
            LineParams random_restart = perturbLineParameters(current_line, current_step * 5.0f);
            double error = objectiveFunction(random_restart);
            perturbations.push_back(random_restart);
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
                stuck_counter = 0;  // 重置停滞计数器

                // 成功改进，减小步长以精细调整
                if (current_step_idx < step_sizes.size() - 1) {
                    current_step_idx++;
                }
            }
        }
        else {
            stuck_counter++;

            if (stuck_counter > max_stuck_iterations) {
                // 多次停滞，增加步长以跳出局部最优
                if (current_step_idx > 0) {
                    current_step_idx--;
                }
                stuck_counter = 0;
            }
        }

        if (iter % 10 == 0 && verbose_) {
            printDebugInfo("LM优化迭代 " + std::to_string(iter) +
                ", 误差: " + std::to_string(best_error) +
                ", 当前步长: " + std::to_string(current_step) +
                ", 停滞计数: " + std::to_string(stuck_counter));
        }

        // 放宽收敛条件
        if (iter > 20 && best_error < tolerance_ * 2.0) {
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

    // 使用多种分布的随机扰动
    std::uniform_real_distribution<float> angle_dist(-1.0f, 1.0f);
    std::normal_distribution<float> pos_dist(0.0f, 0.1f);
    std::cauchy_distribution<float> cauchy_dist(0.0f, 0.5f);  // 柯西分布，更容易跳出局部最优

    int search_iterations = std::min(max_iterations_, 200);

    // 动态调整的步长因子
    float global_step_size = 1.0f;  // 全局步长
    float direction_mix_factor = 0.5f;  // 方向混合因子
    float position_step_factor = 0.1f;  // 位置步长因子

    for (int i = 0; i < search_iterations; ++i) {
        // 动态调整步长：早期大范围搜索，后期精细调整
        float iteration_factor = 1.0f - static_cast<float>(i) / search_iterations;
        float current_global_step = global_step_size * iteration_factor;

        // 生成三种不同策略的候选解
        std::vector<LineParams> candidates;
        std::vector<double> candidate_errors;

        // 策略1: 全局大范围随机搜索（使用均匀分布）
        LineParams candidate1 = best_line;
        Eigen::Vector3f random_dir1(angle_dist(gen), angle_dist(gen), angle_dist(gen));
        random_dir1.normalize();
        candidate1.direction = (best_line.direction * (1.0f - direction_mix_factor) +
            random_dir1 * direction_mix_factor).normalized();
        candidate1.point = best_line.point +
            Eigen::Vector3f(pos_dist(gen), pos_dist(gen), pos_dist(gen)) *
            (position_step_factor * current_global_step);
        candidates.push_back(candidate1);

        // 策略2: 中范围搜索（使用正态分布）
        LineParams candidate2 = best_line;
        Eigen::Vector3f random_dir2(pos_dist(gen), pos_dist(gen), pos_dist(gen));
        random_dir2.normalize();
        candidate2.direction = (best_line.direction * 0.7f + random_dir2 * 0.3f).normalized();
        candidate2.point = best_line.point + random_dir2 * (0.05f * current_global_step);
        candidates.push_back(candidate2);

        // 策略3: 使用柯西分布的大步长扰动
        if (i < search_iterations / 2) {  // 前一半迭代使用大步长探索
            LineParams candidate3 = best_line;
            Eigen::Vector3f cauchy_perturb(cauchy_dist(gen), cauchy_dist(gen), cauchy_dist(gen));
            candidate3.direction = (best_line.direction + cauchy_perturb * 0.1f).normalized();
            candidate3.point = best_line.point + cauchy_perturb * (0.2f * current_global_step);
            candidates.push_back(candidate3);
        }

        // 评估所有候选解
        for (auto& candidate : candidates) {
            double error = objectiveFunction(candidate);

            // 使用模拟退火思想，以一定概率接受稍差的解
            if (error < best_error) {
                best_error = error;
                best_line = candidate;
            }
            else if (i < search_iterations / 4) {  // 前1/4迭代以概率接受较差解
                float temperature = 1.0f - static_cast<float>(i) / (search_iterations / 4);
                float accept_prob = std::exp(-(error - best_error) / (temperature + 1e-6));

                std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);
                if (prob_dist(gen) < accept_prob) {
                    best_error = error;
                    best_line = candidate;
                }
            }
        }

        if (i % 20 == 0 && verbose_) {
            printDebugInfo("随机搜索迭代 " + std::to_string(i) +
                ", 最佳误差: " + std::to_string(best_error) +
                ", 当前步长因子: " + std::to_string(current_global_step));
        }

        // 提前终止条件放宽
        if (i > 50 && best_error < tolerance_ * 5.0) {
            break;
        }
    }

    return best_line;
}
//MeasureCylindricity::LineParams MeasureCylindricity::randomSearchOptimization()
//{
//    LineParams best_line = initializeLineParameters();
//    double best_error = objectiveFunction(best_line);
//
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<float> angle_dist(-1.0f, 1.0f);
//    std::normal_distribution<float> pos_dist(0.0f, 0.1f);
//
//    int search_iterations = std::min(max_iterations_, 200);
//
//    for (int i = 0; i < search_iterations; ++i) {
//        LineParams test_line = best_line;
//
//        Eigen::Vector3f random_vec(angle_dist(gen), angle_dist(gen), angle_dist(gen));
//        random_vec.normalize();
//
//        float mix_factor = 0.3f;
//        test_line.direction = (best_line.direction * (1.0f - mix_factor) +
//            random_vec * mix_factor).normalized();
//
//        test_line.point = best_line.point +
//            Eigen::Vector3f(pos_dist(gen), pos_dist(gen), pos_dist(gen)) * 0.01f;
//
//        double error = objectiveFunction(test_line);
//
//        if (error < best_error) {
//            best_error = error;
//            best_line = test_line;
//        }
//
//        if (i % 40 == 0 && verbose_) {
//            printDebugInfo("随机搜索迭代 " + std::to_string(i) +
//                ", 最佳误差: " + std::to_string(best_error));
//        }
//    }
//
//    return best_line;
//}

namespace {
// 四自由度(方向2+圆心2) Nelder-Mead 单纯形搜索, x0 为初值, 返回最优顶点 xBest
void NelderMead4D(const Eigen::Vector4f& x0,
                  const std::function<double(const Eigen::Vector4f&)>& eval,
                  Eigen::Vector4f& xBest)
{
    const int kDim = 4;
    const Eigen::Vector4f steps(0.1f, 0.1f, 1.0f, 1.0f);
    Eigen::Vector4f simplex[kDim + 1];
    double f[kDim + 1];
    simplex[0] = x0;
    f[0] = eval(x0);
    for (int i = 0; i < kDim; ++i) {
        simplex[i + 1] = x0;
        simplex[i + 1][i] += steps[i];
        f[i + 1] = eval(simplex[i + 1]);
    }

    // 按目标值排序的顶点下标
    int order[kDim + 1] = { 0, 1, 2, 3, 4 };
    auto sortVertices = [&]() {
        for (int i = 0; i <= kDim; ++i) {
            for (int j = i + 1; j <= kDim; ++j) {
                if (f[order[j]] < f[order[i]]) {
                    std::swap(order[i], order[j]);
                }
            }
        }
    };

    const float kAlpha = 1.0f, kGamma = 2.0f, kRho = 0.5f, kSigma = 0.5f;
    const int kMaxIters = 300;
    for (int iter = 0; iter < kMaxIters; ++iter) {
        sortVertices();

        // 收敛判断: 顶点目标值差异小于容差
        double spread = 0.0;
        for (int i = 0; i <= kDim; ++i) {
            spread = std::max(spread, std::abs(f[order[i]] - f[order[0]]));
        }
        if (spread < 1e-8) break;

        // 重心(去掉最差点 order[kDim])
        Eigen::Vector4f xo = Eigen::Vector4f::Zero();
        for (int i = 0; i < kDim; ++i) {
            xo += simplex[order[i]];
        }
        xo /= static_cast<float>(kDim);

        // 反射
        const Eigen::Vector4f xr = xo + (xo - simplex[order[kDim]]) * kAlpha;
        const double fr = eval(xr);

        if (fr < f[order[0]]) {
            // 扩张
            const Eigen::Vector4f xe = xo + (xr - xo) * kGamma;
            const double fe = eval(xe);
            if (fe < fr) {
                simplex[order[kDim]] = xe;
                f[order[kDim]] = fe;
            }
            else {
                simplex[order[kDim]] = xr;
                f[order[kDim]] = fr;
            }
        }
        else if (fr < f[order[kDim - 1]]) {
            simplex[order[kDim]] = xr;
            f[order[kDim]] = fr;
        }
        else {
            if (fr < f[order[kDim]]) {
                // 外部收缩
                const Eigen::Vector4f xc = xo + (xr - xo) * kRho;
                const double fc = eval(xc);
                if (fc <= fr) {
                    simplex[order[kDim]] = xc;
                    f[order[kDim]] = fc;
                    continue;
                }
            }
            else {
                // 内部收缩
                const Eigen::Vector4f xc = xo + (simplex[order[kDim]] - xo) * kRho;
                const double fc = eval(xc);
                if (fc < f[order[kDim]]) {
                    simplex[order[kDim]] = xc;
                    f[order[kDim]] = fc;
                    continue;
                }
            }
            // 整体收缩
            for (int i = 1; i <= kDim; ++i) {
                simplex[order[i]] = simplex[order[0]]
                    + (simplex[order[i]] - simplex[order[0]]) * kSigma;
                f[order[i]] = eval(simplex[order[i]]);
            }
        }
    }

    // 取最优顶点写回
    sortVertices();
    xBest = simplex[order[0]];
}
} // namespace

// ============================================================
// 第二阶段(改进): 粗到精确定性搜索
// 思路: 轴线方向仅2个自由度(单位球面上的点). 在球面上做 Fibonacci 均匀采样,
//       对每个候选方向在其垂直平面内做固定半径圆心拟合(代数最小二乘初值 +
//       Gauss-Newton 精化), 再取残差最小的若干候选做四自由度(方向2+圆心2)
//       Nelder-Mead 联合精化, 稳定收敛到几何残差的全局最优,
//       避免扰动搜索在小弧段退化方向上的收敛停滞.
// ============================================================
std::vector<Eigen::Vector3f> MeasureCylindricity::sampleDirectionsSphere(int n)
{
    std::vector<Eigen::Vector3f> dirs;
    dirs.reserve(static_cast<size_t>(n));

    // Fibonacci 球面采样: 黄金角 = pi * (3 - sqrt(5))
    const double kGoldenAngle = 3.14159265358979323846 * (3.0 - 2.23606797749978969641);
    for (int i = 0; i < n; ++i) {
        const double y = 1.0 - 2.0 * (i + 0.5) / n;
        const double rad = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double th = kGoldenAngle * i;
        dirs.emplace_back(static_cast<float>(std::cos(th) * rad),
                          static_cast<float>(std::sin(th) * rad),
                          static_cast<float>(y));
    }
    return dirs;
}

double MeasureCylindricity::evaluateCandidate(const Eigen::Vector3f& center,
                                              const Eigen::Vector3f& direction)
{
    LineParams line;
    line.point = center;
    line.direction = direction.normalized();
    return objectiveFunction(line);
}

void MeasureCylindricity::fitCircleCenterFixedRadius(const Eigen::Vector3f& direction,
                                                     const Eigen::Vector3f& c0,
                                                     Eigen::Vector2f& ab) const
{
    const Eigen::Vector3f d = direction.normalized();
    const Eigen::Vector3f t1 = d.unitOrthogonal();
    const Eigen::Vector3f t2 = d.cross(t1);

    // 投影统计(同时为代数初值与 Gauss-Newton 准备)
    double su = 0.0, sv = 0.0, suu = 0.0, svv = 0.0, suv = 0.0;
    double su3 = 0.0, sv3 = 0.0, suv2 = 0.0, svu2 = 0.0, sr2 = 0.0;
    int n = 0;
    for (const auto& p : input_cloud_->points) {
        if (!isPointValid(p)) continue;
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const Eigen::Vector3f v = pt - c0;
        const double u = static_cast<double>(v.dot(t1));
        const double w = static_cast<double>(v.dot(t2));
        const double uu = u * u;
        const double ww = w * w;
        su += u; sv += w;
        suu += uu; svv += ww;
        suv += u * w;
        su3 += uu * u; sv3 += ww * w;
        suv2 += u * ww; svu2 += w * uu;
        sr2 += uu + ww;
        ++n;
    }
    if (n < 3) {
        ab.setZero();
        return;
    }

    // 代数最小二乘初值: 最小化 sum(((u-a)^2+(w-b)^2 - r^2)^2), 未知量 [a, b, s]
    Eigen::Matrix3d M;
    M << 4.0 * suu, 4.0 * suv, -2.0 * su,
         4.0 * suv, 4.0 * svv, -2.0 * sv,
         -2.0 * su, -2.0 * sv, static_cast<double>(n);
    Eigen::Vector3d rhs;
    rhs << 2.0 * (su3 + suv2), 2.0 * (sv3 + svu2), -sr2;
    const Eigen::Vector3d x = M.fullPivLu().solve(rhs);
    double a = x[0];
    double b = x[1];

    // Gauss-Newton 精化(几何残差)
    const double r = design_radius_;
    for (int iter = 0; iter < 10; ++iter) {
        double h00 = 0.0, h01 = 0.0, h11 = 0.0, g0 = 0.0, g1 = 0.0;
        for (const auto& p : input_cloud_->points) {
            if (!isPointValid(p)) continue;
            const Eigen::Vector3f pt(p.x, p.y, p.z);
            const Eigen::Vector3f v = pt - c0;
            const double u = static_cast<double>(v.dot(t1)) - a;
            const double w = static_cast<double>(v.dot(t2)) - b;
            const double rho = std::max(std::sqrt(u * u + w * w), 1e-9);
            const double res = rho - r;
            const double ja = -u / rho;
            const double jb = -w / rho;
            h00 += ja * ja; h01 += ja * jb; h11 += jb * jb;
            g0 += ja * res; g1 += jb * res;
        }
        const double det = h00 * h11 - h01 * h01;
        if (std::abs(det) < 1e-12) break;
        const double da = (h01 * g1 - h11 * g0) / det;
        const double db = (h01 * g0 - h00 * g1) / det;
        a += da; b += db;
        if (std::hypot(da, db) < 1e-4) break;
    }

    ab = Eigen::Vector2f(static_cast<float>(a), static_cast<float>(b));
}

void MeasureCylindricity::refineLineParams(const Eigen::Vector3f& c0, LineParams& line)
{
    // 参数化: 方向 = (sinφcosθ, sinφsinθ, cosφ); 圆心 = c0 + a*t1 + b*t2
    const Eigen::Vector3f d0 = line.direction.normalized();
    const Eigen::Vector3f t1 = d0.unitOrthogonal();
    const Eigen::Vector3f t2 = d0.cross(t1);
    Eigen::Vector4f x;
    x[0] = std::atan2(d0.y(), d0.x());
    x[1] = std::acos(std::max(-1.0f, std::min(1.0f, d0.z())));
    x[2] = (line.point - c0).dot(t1);
    x[3] = (line.point - c0).dot(t2);

    auto eval = [&](const Eigen::Vector4f& v) -> double {
        const float th = v[0];
        const float ph = v[1];
        const float sinp = std::sin(ph);
        const Eigen::Vector3f dir(std::cos(th) * sinp, std::sin(th) * sinp, std::cos(ph));
        const Eigen::Vector3f tt1 = dir.unitOrthogonal();
        const Eigen::Vector3f tt2 = dir.cross(tt1);
        const Eigen::Vector3f center = c0 + tt1 * v[2] + tt2 * v[3];
        return evaluateCandidate(center, dir);
    };

    // Nelder-Mead 单纯形搜索(公共实现见本文件匿名命名空间)
    NelderMead4D(x, eval, x);

    const float th = x[0];
    const float ph = x[1];
    const float sinp = std::sin(ph);
    line.direction = Eigen::Vector3f(std::cos(th) * sinp, std::sin(th) * sinp,
                                     std::cos(ph)).normalized();
    const Eigen::Vector3f tt1 = line.direction.unitOrthogonal();
    const Eigen::Vector3f tt2 = line.direction.cross(tt1);
    line.point = c0 + tt1 * x[2] + tt2 * x[3];
}

MeasureCylindricity::LineParams MeasureCylindricity::coarseToFineOptimization()
{
    const Eigen::Vector3f c0 = computeCentroid();
    const int kGridSize = 400;
    const int kTopK = 8;

    printDebugInfo("第二阶段粗到精搜索: 球面采样 " + std::to_string(kGridSize)
        + " 个候选方向...");

    const std::vector<Eigen::Vector3f> dirs = sampleDirectionsSphere(kGridSize);

    // 粗搜索: 每个方向配固定半径圆心, 评估几何残差
    struct Candidate {
        double error;
        Eigen::Vector3f center;
        Eigen::Vector3f direction;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(dirs.size());
    for (const auto& d : dirs) {
        Eigen::Vector2f ab;
        fitCircleCenterFixedRadius(d, c0, ab);
        const Eigen::Vector3f t1 = d.unitOrthogonal();
        const Eigen::Vector3f t2 = d.cross(t1);
        const Eigen::Vector3f center = c0 + t1 * ab.x() + t2 * ab.y();
        Candidate cand;
        cand.direction = d;
        cand.center = center;
        cand.error = evaluateCandidate(center, d);
        candidates.push_back(cand);
    }

    // 取残差最小的 kTopK 个候选
    std::partial_sort(candidates.begin(), candidates.begin() + kTopK, candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.error < b.error; });

    // 局部精化: 对 top-K 做四自由度联合精化, 取最优
    double bestError = candidates.front().error;
    LineParams best;
    best.point = candidates.front().center;
    best.direction = candidates.front().direction;
    for (int i = 0; i < kTopK && i < static_cast<int>(candidates.size()); ++i) {
        LineParams line;
        line.point = candidates[i].center;
        line.direction = candidates[i].direction;
        refineLineParams(c0, line);
        const double err = evaluateCandidate(line.point, line.direction);
        if (err < bestError) {
            bestError = err;
            best = line;
        }
    }

    printDebugInfo("第二阶段优化完成, 误差: " + std::to_string(bestError));
    return best;
}

// ============================================================
// 第三阶段(新增): 焊缝带检测与轴向约束修正
// 思路: 利用第二阶段拟合的残差场检测焊缝突起(稳健阈值+欧氏聚类+主方向),
//       再以"焊缝点轴向坐标方差"为软约束做局部精化, 使轴线垂直于焊缝走向,
//       补足小弧段在退化方向上的信息缺失. 未检测到焊缝时自动回退.
// ============================================================
bool MeasureCylindricity::detectWeldBand(const LineParams& line,
                                         std::vector<int>& weldIndices,
                                         Eigen::Vector3f& weldDirection)
{
    weldIndices.clear();
    const Eigen::Vector3f d = line.direction.normalized();

    // 1. 逐点带符号径向偏差(向外为正)
    std::vector<double> devs;
    std::vector<int> cloudIdxs;
    devs.reserve(input_cloud_->size());
    cloudIdxs.reserve(input_cloud_->size());
    for (std::size_t i = 0; i < input_cloud_->size(); ++i) {
        const auto& p = input_cloud_->points[i];
        if (!isPointValid(p)) continue;
        const double dist = computeDistanceToLine(Eigen::Vector3f(p.x, p.y, p.z), line);
        devs.push_back(dist - design_radius_);
        cloudIdxs.push_back(static_cast<int>(i));
    }
    const int n = static_cast<int>(devs.size());
    if (n < 200) return false;

    // 2. 稳健阈值: median + 3 * 1.4826 * MAD, 下限 0.8mm
    std::vector<double> sorted = devs;
    std::nth_element(sorted.begin(), sorted.begin() + n / 2, sorted.end());
    const double med = sorted[n / 2];
    std::vector<double> absDev(n);
    for (int i = 0; i < n; ++i) absDev[i] = std::abs(devs[i] - med);
    std::nth_element(absDev.begin(), absDev.begin() + n / 2, absDev.end());
    double thr = med + weld_thr_factor_ * 1.4826 * absDev[n / 2];
    if (thr < 0.8) thr = 0.8;

    // 3. 候选突起点的欧氏聚类
    pcl::PointCloud<pcl::PointXYZ>::Ptr cand(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> candToCloud;
    cand->points.reserve(static_cast<std::size_t>(n));
    candToCloud.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        if (devs[i] > thr) {
            cand->points.push_back(input_cloud_->points[cloudIdxs[i]]);
            candToCloud.push_back(cloudIdxs[i]);
        }
    }
    cand->width = cand->points.size();
    cand->height = 1;
    cand->is_dense = true;
    if (cand->size() < 100) return false;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cand);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(weld_cluster_tolerance_);
    ec.setMinClusterSize(weld_min_cluster_size_);
    ec.setMaxClusterSize(static_cast<int>(cand->size() / 2) + 1);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cand);
    ec.extract(clusters);
    if (clusters.empty()) return false;

    // 取最大簇作为焊缝带候选
    const pcl::PointIndices* bestCluster = &clusters[0];
    for (const auto& cl : clusters) {
        if (cl.indices.size() > bestCluster->indices.size()) bestCluster = &cl;
    }
    if (bestCluster->indices.size() < 100) return false;

    // 4. 主方向: 最大簇坐标 PCA 的最大特征向量
    pcl::PointCloud<pcl::PointXYZ>::Ptr band(new pcl::PointCloud<pcl::PointXYZ>);
    band->points.reserve(bestCluster->indices.size());
    for (int k : bestCluster->indices) band->points.push_back(cand->points[k]);
    band->width = band->points.size();
    band->height = 1;
    band->is_dense = true;
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(band);
    const Eigen::Vector3f evals = pca.getEigenValues();
    const Eigen::Matrix3f evecs = pca.getEigenVectors();
    int maxIdx = 0;
    for (int i = 1; i < 3; ++i) {
        if (evals(i) > evals(maxIdx)) maxIdx = i;
    }
    weldDirection = evecs.col(maxIdx);
    weldDirection.normalize();

    // 拉长性检查: 最大/次大特征值比, 排除团状噪声
    const double lamMax = evals(maxIdx);
    double lamSecond = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (i != maxIdx) lamSecond = std::max(lamSecond, static_cast<double>(evals(i)));
    }
    if (lamSecond <= 0.0 || lamMax / lamSecond < 1.5) return false;

    // 5. 输出焊缝点索引
    weldIndices.reserve(bestCluster->indices.size());
    for (int k : bestCluster->indices) weldIndices.push_back(candToCloud[k]);
    return true;
}

double MeasureCylindricity::objectiveWithWeldConstraint(const LineParams& line,
                                                        const std::vector<int>& weldIndices)
{
    // 目标 = 全体点的径向偏差均方 + λ × 焊缝点轴向坐标方差
    const double residual = objectiveFunction(line);
    if (weldIndices.empty()) return residual;

    const Eigen::Vector3f d = line.direction.normalized();
    double sumZ = 0.0;
    double sumZ2 = 0.0;
    int n = 0;
    for (int idx : weldIndices) {
        const auto& p = input_cloud_->points[idx];
        if (!isPointValid(p)) continue;
        const double z = static_cast<double>(
            (Eigen::Vector3f(p.x, p.y, p.z) - line.point).dot(d));
        sumZ += z;
        sumZ2 += z * z;
        ++n;
    }
    if (n < 2) return residual;

    const double mean = sumZ / n;
    const double var = sumZ2 / n - mean * mean;
    return residual + weld_lambda_ * var;
}

void MeasureCylindricity::refineLineParamsWithWeld(const Eigen::Vector3f& c0,
                                                   LineParams& line,
                                                   const std::vector<int>& weldIndices)
{
    // 参数化: 方向 = (sinφcosθ, sinφsinθ, cosφ); 圆心 = c0 + a*t1 + b*t2
    const Eigen::Vector3f d0 = line.direction.normalized();
    const Eigen::Vector3f t1 = d0.unitOrthogonal();
    const Eigen::Vector3f t2 = d0.cross(t1);
    Eigen::Vector4f x;
    x[0] = std::atan2(d0.y(), d0.x());
    x[1] = std::acos(std::max(-1.0f, std::min(1.0f, d0.z())));
    x[2] = (line.point - c0).dot(t1);
    x[3] = (line.point - c0).dot(t2);

    auto eval = [&](const Eigen::Vector4f& v) -> double {
        const float th = v[0];
        const float ph = v[1];
        const float sinp = std::sin(ph);
        const Eigen::Vector3f dir(std::cos(th) * sinp, std::sin(th) * sinp, std::cos(ph));
        const Eigen::Vector3f tt1 = dir.unitOrthogonal();
        const Eigen::Vector3f tt2 = dir.cross(tt1);
        const Eigen::Vector3f center = c0 + tt1 * v[2] + tt2 * v[3];
        LineParams cand;
        cand.point = center;
        cand.direction = dir;
        return objectiveWithWeldConstraint(cand, weldIndices);
    };

    // 局部精化(含焊缝轴向方差惩罚)
    NelderMead4D(x, eval, x);

    const float th = x[0];
    const float ph = x[1];
    const float sinp = std::sin(ph);
    line.direction = Eigen::Vector3f(std::cos(th) * sinp, std::sin(th) * sinp,
                                     std::cos(ph)).normalized();
    const Eigen::Vector3f tt1 = line.direction.unitOrthogonal();
    const Eigen::Vector3f tt2 = line.direction.cross(tt1);
    line.point = c0 + tt1 * x[2] + tt2 * x[3];
}

MeasureCylindricity::LineParams MeasureCylindricity::optimizeWithWeldConstraint(
    const LineParams& baseLine)
{
    weld_points_->clear();
    weld_constraint_applied_ = false;

    std::vector<int> weldIndices;
    Eigen::Vector3f weldDirection;
    if (!detectWeldBand(baseLine, weldIndices, weldDirection)) {
        printDebugInfo("焊缝约束: 未检测到显著焊缝带, 保持第二阶段结果");
        return baseLine;
    }

    // 方向合理性检查: 周向焊缝走向应与轴线近似垂直, 否则不施加约束
    const double cosAngle = std::abs(
        weldDirection.dot(baseLine.direction.normalized()));
    const double angleDeg = std::acos(std::max(-1.0, std::min(1.0, cosAngle)))
        * 180.0 / 3.14159265358979323846;
    printDebugInfo("焊缝约束: 检测到 " + std::to_string(weldIndices.size())
        + " 个焊缝点, 焊缝走向与轴线夹角 " + std::to_string(angleDeg) + "°");
    if (angleDeg < 60.0) {
        printDebugInfo("焊缝约束: 走向与轴线夹角过小, 判定为非周向焊缝, 不施加约束");
        return baseLine;
    }

    // 填充焊缝点云(供可视化)
    weld_points_->points.reserve(weldIndices.size());
    for (int idx : weldIndices) weld_points_->points.push_back(input_cloud_->points[idx]);
    weld_points_->width = weld_points_->points.size();
    weld_points_->height = 1;
    weld_points_->is_dense = true;

    // 含焊缝约束的局部精化
    LineParams refined = baseLine;
    refineLineParamsWithWeld(computeCentroid(), refined, weldIndices);
    weld_constraint_applied_ = true;
    printDebugInfo("焊缝约束: 轴向修正完成");
    return refined;
}

MeasureCylindricity::LineParams MeasureCylindricity::optimizeLineParameters()
{
    printDebugInfo("开始优化直线参数（4自由度, 粗到精确定性搜索）...");
    return coarseToFineOptimization();
}

MeasureCylindricity::AssessmentResult MeasureCylindricity::evaluateCylindricityWithWeld()
{
    AssessmentResult result;

    if (!isValidPointCloud()) {
        result.assessment_message = "错误: 点云无效";
        return result;
    }

    if (design_radius_ <= 0) {
        result.assessment_message = "错误: 设计半径必须为正数";
        return result;
    }

    printDebugInfo("开始圆柱度评估（含焊缝约束）...");
    printDebugInfo("点云点数: " + std::to_string(input_cloud_->size()));
    printDebugInfo("设计半径: " + std::to_string(design_radius_));
    printDebugInfo("容差阈值: " + std::to_string(tolerance_));

    try {
        // 第二阶段粗到精优化 + 第三阶段焊缝约束修正
        LineParams best_line = optimizeLineParameters();
        best_line = optimizeWithWeldConstraint(best_line);

        result = computeAssessmentMetrics(best_line);

        // 新增：生成热力图点云
        generateHeatMapCloud(best_line);

        std::stringstream message;
        message << "=== 圆柱度评估报告（含焊缝约束）===\n";
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

        if (weld_constraint_applied_) {
            message << "焊缝约束: 检测到 " << getWeldPointCount()
                << " 个焊缝点，已完成轴向修正。\n";
        }
        else {
            message << "焊缝约束: 未检测到显著焊缝带，保持无约束结果。\n";
        }

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

    int num_points = static_cast<int>(input_cloud_->size());
    double sum_distance = 0.0;
    double sum_squared_distance = 0.0;
    double sum_squared_deviation = 0.0;
    int outlier_count = 0;
    int valid_points = 0;

    // 使用固定大小的vector存储偏差值
    std::vector<double> deviations(num_points, std::numeric_limits<double>::max());
    std::vector<bool> is_valid_point(num_points, false);

    // 第一阶段：并行计算所有点的距离
#pragma omp parallel for
    for (int i = 0; i < num_points; ++i) {
        const auto& p = input_cloud_->points[i];
        if (!isPointValid(p)) continue;

        Eigen::Vector3f point(p.x, p.y, p.z);
        double distance_to_axis = computeDistanceToLine(point, line);
        deviations[i] = std::abs(distance_to_axis - design_radius_);
        is_valid_point[i] = true;
    }

    // 第二阶段：串行统计（避免竞争）
    result.min_deviation = std::numeric_limits<double>::max();
    result.max_deviation = 0.0;

    for (int i = 0; i < num_points; ++i) {
        if (!is_valid_point[i]) continue;

        double deviation = deviations[i];
        distance_map_.push_back(deviation);
        sum_distance += deviation;
        sum_squared_distance += deviation * deviation;
        valid_points++;

        if (deviation > tolerance_) {
            outlier_count++;
            outliers_->push_back(input_cloud_->points[i]);
        }
        else {
            inliers_->push_back(input_cloud_->points[i]);
        }

        // 更新最小最大偏差
        if (deviation < result.min_deviation) {
            result.min_deviation = deviation;
        }
        if (deviation > result.max_deviation) {
            result.max_deviation = deviation;
        }
    }

    // 第三阶段：计算统计指标
    if (valid_points > 0) {
        result.outlier_ratio = static_cast<double>(outlier_count) / valid_points;
        result.mean_deviation = sum_distance / valid_points;
        result.rms_error = std::sqrt(sum_squared_distance / valid_points);

        // 计算均匀性（标准差）
        for (int i = 0; i < num_points; ++i) {
            if (!is_valid_point[i]) continue;
            double dev = deviations[i];
            sum_squared_deviation += std::pow(dev - result.mean_deviation, 2);
        }
        result.uniformity_rms = std::sqrt(sum_squared_deviation / valid_points);

        result.is_acceptable = (result.outlier_ratio < 0.1);
        result.best_fit_line = line;
    }
    else {
        result.min_deviation = 0.0;
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

        // ============ 修复1: 生成热力图点云 ============
        generateHeatMapCloud(given_line);
        // =============================================

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
        // ============ 修复2: 修正输出文本的单位笔误 (m -> mm) ============
        message << "平均偏差: " << std::setprecision(4) << result.mean_deviation << " mm\n";
        message << "RMS偏差: " << std::setprecision(4) << result.rms_error << " mm\n\n";
        // ===============================================================
        message << "均匀性(标准差): " << result.uniformity_rms << " mm\n\n";

        message << (result.is_acceptable ? "✅ 圆柱度良好" : "❌ 圆柱度不佳");
        message << "（不贴合点比例" << (result.is_acceptable ? "＜" : "≥") << "10%）";

        result.assessment_message = message.str();

        // 保存最后一次评估结果
        last_assessment_result_ = result;
        // ============ 修复3: 保存本次评估所使用的直线参数 ============
        last_optimized_line_ = given_line; // 将“给定”的直线视为本次的“最优”结果进行保存
        // ============================================================

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
