#include "Measure/MeasureWeldHeight.h"
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <limits>
#include <omp.h>

// ============ 构造/析构 ============

MeasureWeldHeight::MeasureWeldHeight()
    : search_radius_(20.0)
    , region_size_(10.0)
    , ransac_threshold_(0.5)
    , min_neighbors_(15)
    , verbose_(true)
    , min_height_(0.0)
    , max_height_(0.0)
    , max_abs_height_(0.0)
{
    weld_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    base_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    base_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    heatmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

MeasureWeldHeight::~MeasureWeldHeight() {}

// ============ 参数设置 ============

void MeasureWeldHeight::setWeldCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud && !cloud->empty()) {
        weld_cloud_ = cloud;
        printDebug("焊缝点云设置完成，点数: " + std::to_string(weld_cloud_->size()));
    } else {
        printDebug("错误: 焊缝点云为空或无效");
    }
}

void MeasureWeldHeight::setBaseCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud && !cloud->empty()) {
        base_cloud_ = cloud;
        base_kdtree_->setInputCloud(base_cloud_);
        printDebug("壁面点云设置完成，点数: " + std::to_string(base_cloud_->size()));
    } else {
        printDebug("错误: 壁面点云为空或无效");
    }
}

void MeasureWeldHeight::setSearchRadius(double radius_mm)
{
    if (radius_mm > 0) {
        search_radius_ = radius_mm;
        printDebug("搜索半径设置为: " + std::to_string(search_radius_) + " mm");
    } else {
        printDebug("错误: 搜索半径必须为正数");
    }
}

void MeasureWeldHeight::setRegionSize(double size_mm)
{
    if (size_mm > 0) {
        region_size_ = size_mm;
        printDebug("网格边长设置为: " + std::to_string(region_size_) + " mm");
    } else {
        printDebug("错误: 网格边长必须为正数");
    }
}

void MeasureWeldHeight::setRansacThreshold(double threshold_mm)
{
    if (threshold_mm >= 0) {
        ransac_threshold_ = threshold_mm;
        printDebug("RANSAC阈值设置为: " + std::to_string(ransac_threshold_) + " mm");
    } else {
        printDebug("错误: RANSAC阈值必须为非负数");
    }
}

void MeasureWeldHeight::setMinNeighbors(int min_n)
{
    if (min_n >= 3) {
        min_neighbors_ = min_n;
        printDebug("最少邻域点数设置为: " + std::to_string(min_neighbors_));
    } else {
        printDebug("错误: 最少邻域点数至少为3");
    }
}

void MeasureWeldHeight::setVerbose(bool verbose) { verbose_ = verbose; }

// ============ 主入口 ============

MeasureWeldHeight::AssessmentResult MeasureWeldHeight::evaluate()
{
    AssessmentResult result;

    if (!weld_cloud_ || weld_cloud_->empty()) {
        result.assessment_message = "错误: 焊缝点云为空";
        return result;
    }
    if (!base_cloud_ || base_cloud_->empty()) {
        result.assessment_message = "错误: 壁面点云为空";
        return result;
    }

    printDebug("========== 开始焊缝高度评估 ==========");
    printDebug("焊缝点数: " + std::to_string(weld_cloud_->size()));
    printDebug("壁面点数: " + std::to_string(base_cloud_->size()));
    printDebug("搜索半径: " + std::to_string(search_radius_) + " mm");
    printDebug("网格边长: " + std::to_string(region_size_) + " mm");

    try {
        // 步骤1: 空间网格划分
        buildGrid();
        printDebug("网格划分完成，有效网格数: " + std::to_string(grid_cells_.size()));

        // 步骤2: 每格拟合局部平面
        fitLocalPlanes();

        // 步骤3: 逐点计算高度
        measureAllPoints();

        // 生成热力图
        generateHeatMap();

        // 汇总结果
        result = last_result_;
        result.assessment_message = last_result_.assessment_message;

    } catch (const std::exception& e) {
        result.assessment_message = "评估过程出错: " + std::string(e.what());
        printDebug(result.assessment_message);
    }

    printDebug("========== 评估完成 ==========");
    return result;
}

// ============ 步骤1: 空间网格划分 ============

void MeasureWeldHeight::buildGrid()
{
    grid_cells_.clear();

    // 计算焊缝点云包围盒
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*weld_cloud_, min_pt, max_pt);

    // 扩展边界半个搜索半径，确保边缘格也能搜到壁面点
    float margin = static_cast<float>(search_radius_ * 0.5);
    min_pt.x -= margin; min_pt.y -= margin; min_pt.z -= margin;
    max_pt.x += margin; max_pt.y += margin; max_pt.z += margin;

    grid_origin_ = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);

    // 计算网格维度
    float dx = max_pt.x - min_pt.x;
    float dy = max_pt.y - min_pt.y;
    float dz = max_pt.z - min_pt.z;
    float cell = static_cast<float>(region_size_);

    int nx = std::max(1, static_cast<int>(std::ceil(dx / cell)));
    int ny = std::max(1, static_cast<int>(std::ceil(dy / cell)));
    int nz = std::max(1, static_cast<int>(std::ceil(dz / cell)));

    grid_dims_ = Eigen::Vector3i(nx, ny, nz);
    cell_size_vec_ = Eigen::Vector3f(cell, cell, cell);

    // 初始化网格：用 map 映射 (ix, iy, iz) -> cell index
    // 为节省内存，只保存非空格
    std::map<std::tuple<int,int,int>, int> cell_index_map;
    int cell_count = 0;

    for (const auto& pt : weld_cloud_->points) {
        if (!isPointValid(pt)) continue;

        int ix = static_cast<int>(std::floor((pt.x - grid_origin_.x()) / cell));
        int iy = static_cast<int>(std::floor((pt.y - grid_origin_.y()) / cell));
        int iz = static_cast<int>(std::floor((pt.z - grid_origin_.z()) / cell));

        // 边界保护
        ix = std::max(0, std::min(ix, nx - 1));
        iy = std::max(0, std::min(iy, ny - 1));
        iz = std::max(0, std::min(iz, nz - 1));

        auto key = std::make_tuple(ix, iy, iz);
        auto it = cell_index_map.find(key);
        if (it == cell_index_map.end()) {
            cell_index_map[key] = cell_count;
            GridCell gc;
            gc.index = Eigen::Vector3i(ix, iy, iz);
            gc.weld_points->push_back(pt);
            grid_cells_.push_back(gc);
            cell_count++;
        } else {
            grid_cells_[it->second].weld_points->push_back(pt);
        }
    }

    printDebug("非空格网数: " + std::to_string(grid_cells_.size()) +
        " / 总格数: " + std::to_string(nx * ny * nz));
}

// ============ 步骤2: 每格拟合局部平面 ============

void MeasureWeldHeight::fitLocalPlanes()
{
    int valid_count = 0;

    for (auto& cell : grid_cells_) {
        if (cell.weld_points->empty()) continue;

        // 计算该格焊缝点的质心
        cell.weld_centroid = computeCloudCentroid(cell.weld_points);

        // 以该质心为搜索中心，在壁面点云中找邻域点
        pcl::PointXYZ search_center;
        search_center.x = cell.weld_centroid.x();
        search_center.y = cell.weld_centroid.y();
        search_center.z = cell.weld_centroid.z();

        std::vector<int> indices;
        std::vector<float> distances;
        int found = base_kdtree_->radiusSearch(search_center, static_cast<float>(search_radius_),
            indices, distances);

        if (found < min_neighbors_) {
            cell.plane_valid = false;
            continue;
        }

        // 提取邻域壁面点
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_base(new pcl::PointCloud<pcl::PointXYZ>);
        local_base->reserve(found);
        for (int idx : indices) {
            local_base->push_back(base_cloud_->points[idx]);
        }

        cell.base_centroid = computeCloudCentroid(local_base);

        // RANSAC 拟合局部平面
        cell.plane_coeffs = fitPlaneRANSAC(local_base);

        // 方向归一化：使焊缝点在平面"外侧"为正值
        orientPlaneNormal(cell.plane_coeffs, cell.weld_centroid, cell.base_centroid);

        cell.plane_valid = true;
        valid_count++;
    }

    printDebug("有效平面拟合格数: " + std::to_string(valid_count) +
        " / " + std::to_string(grid_cells_.size()));
}

// ============ 步骤3: 逐点计算高度 ============

void MeasureWeldHeight::measureAllPoints()
{
    min_height_ = std::numeric_limits<double>::max();
    max_height_ = std::numeric_limits<double>::lowest();
    double sum_height = 0.0;
    double sum_sq = 0.0;
    int valid_pts = 0;
    int total_pts = 0;

    // 最高/最低点
    double best_high = std::numeric_limits<double>::lowest();
    double best_low = std::numeric_limits<double>::max();
    pcl::PointXYZ pt_high, pt_low;

    // 用 map 快速查找点所属的 cell: (ix, iy, iz) -> cell index
    std::map<std::tuple<int,int,int>, int> cell_map;
    for (size_t i = 0; i < grid_cells_.size(); ++i) {
        auto& gc = grid_cells_[i];
        cell_map[std::make_tuple(gc.index.x(), gc.index.y(), gc.index.z())] = static_cast<int>(i);
    }

    float cell = static_cast<float>(region_size_);
    int nx = grid_dims_.x();
    int ny = grid_dims_.y();
    int nz = grid_dims_.z();

    for (const auto& pt : weld_cloud_->points) {
        total_pts++;
        if (!isPointValid(pt)) continue;

        // 确定所属网格
        int ix = static_cast<int>(std::floor((pt.x - grid_origin_.x()) / cell));
        int iy = static_cast<int>(std::floor((pt.y - grid_origin_.y()) / cell));
        int iz = static_cast<int>(std::floor((pt.z - grid_origin_.z()) / cell));
        ix = std::max(0, std::min(ix, nx - 1));
        iy = std::max(0, std::min(iy, ny - 1));
        iz = std::max(0, std::min(iz, nz - 1));

        auto it = cell_map.find(std::make_tuple(ix, iy, iz));
        if (it == cell_map.end()) continue;  // 该格无数据，跳过

        auto& cell = grid_cells_[it->second];
        if (!cell.plane_valid) continue;     // 该格无有效平面

        double dist = signedDistanceToPlane(pt, cell.plane_coeffs);
        valid_pts++;

        sum_height += dist;
        sum_sq += dist * dist;

        if (dist < min_height_) min_height_ = dist;
        if (dist > max_height_) max_height_ = dist;

        if (dist > best_high) {
            best_high = dist;
            pt_high = pt;
        }
        if (dist < best_low) {
            best_low = dist;
            pt_low = pt;
        }
    }

    max_abs_height_ = std::max(std::abs(max_height_), std::abs(min_height_));
    if (max_abs_height_ < 1e-10) max_abs_height_ = 1e-5;

    // 填充结果
    if (valid_pts == 0) {
        last_result_.min_height = 0.0;
        last_result_.max_height = 0.0;
        last_result_.mean_height = 0.0;
        last_result_.rms_height = 0.0;
        last_result_.std_height = 0.0;
        last_result_.valid_points = 0;
        last_result_.total_points = total_pts;
        last_result_.assessment_message = "错误: 无有效测量点，请检查搜索半径和网格参数";
        max_abs_height_ = 1e-5;
        printDebug(last_result_.assessment_message);
        return;
    }

    last_result_.min_height = min_height_;
    last_result_.max_height = max_height_;
    last_result_.mean_height = valid_pts > 0 ? sum_height / valid_pts : 0.0;
    last_result_.rms_height = valid_pts > 0 ? std::sqrt(sum_sq / valid_pts) : 0.0;
    last_result_.highest_point = pt_high;
    last_result_.highest_value = best_high;
    last_result_.lowest_point = pt_low;
    last_result_.lowest_value = best_low;
    last_result_.valid_points = valid_pts;
    last_result_.total_points = total_pts;

    // 计算标准差
    if (valid_pts > 1) {
        double variance = (sum_sq / valid_pts) -
            (sum_height / valid_pts) * (sum_height / valid_pts);
        last_result_.std_height = std::sqrt(std::max(0.0, variance));
    }

    // 生成报告文本
    std::stringstream ss;
    ss << "=== 焊缝高度评估报告 ===\n";
    ss << "焊缝总点数: " << total_pts << "\n";
    ss << "有效评估点数: " << valid_pts << "\n\n";
    ss << "最高凸起: +" << std::fixed << std::setprecision(3) << best_high << " mm";
    ss << "  at (" << std::setprecision(1) << pt_high.x << ", "
       << pt_high.y << ", " << pt_high.z << ")\n";
    ss << "最深凹陷: " << std::setprecision(3) << best_low << " mm";
    ss << "  at (" << std::setprecision(1) << pt_low.x << ", "
       << pt_low.y << ", " << pt_low.z << ")\n\n";
    ss << "平均高度: " << std::setprecision(3) << last_result_.mean_height << " mm\n";
    ss << "RMS高度:  " << last_result_.rms_height << " mm\n";
    ss << "标准差:   " << last_result_.std_height << " mm\n";
    ss << "高度范围: [" << min_height_ << ", " << max_height_ << "] mm";

    last_result_.assessment_message = ss.str();
    printDebug(ss.str());
}

// ============ 热力图生成 ============

void MeasureWeldHeight::generateHeatMap()
{
    heatmap_cloud_->clear();
    heatmap_cloud_->width = weld_cloud_->width;
    heatmap_cloud_->height = weld_cloud_->height;
    heatmap_cloud_->is_dense = weld_cloud_->is_dense;
    heatmap_cloud_->points.resize(weld_cloud_->size());

    // 建立 cell map
    std::map<std::tuple<int,int,int>, int> cell_map;
    for (size_t i = 0; i < grid_cells_.size(); ++i) {
        auto& gc = grid_cells_[i];
        cell_map[std::make_tuple(gc.index.x(), gc.index.y(), gc.index.z())] = static_cast<int>(i);
    }

    float cell = static_cast<float>(region_size_);
    int nx = grid_dims_.x();
    int ny = grid_dims_.y();
    int nz = grid_dims_.z();

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(weld_cloud_->size()); ++i) {
        const auto& pt = weld_cloud_->points[i];
        heatmap_cloud_->points[i].x = pt.x;
        heatmap_cloud_->points[i].y = pt.y;
        heatmap_cloud_->points[i].z = pt.z;

        if (!isPointValid(pt)) {
            heatmap_cloud_->points[i].r = 128;
            heatmap_cloud_->points[i].g = 128;
            heatmap_cloud_->points[i].b = 128;
            continue;
        }

        int ix = static_cast<int>(std::floor((pt.x - grid_origin_.x()) / cell));
        int iy = static_cast<int>(std::floor((pt.y - grid_origin_.y()) / cell));
        int iz = static_cast<int>(std::floor((pt.z - grid_origin_.z()) / cell));
        ix = std::max(0, std::min(ix, nx - 1));
        iy = std::max(0, std::min(iy, ny - 1));
        iz = std::max(0, std::min(iz, nz - 1));

        auto it = cell_map.find(std::make_tuple(ix, iy, iz));
        if (it == cell_map.end() || !grid_cells_[it->second].plane_valid) {
            // 无有效平面，标记为灰色
            heatmap_cloud_->points[i].r = 128;
            heatmap_cloud_->points[i].g = 128;
            heatmap_cloud_->points[i].b = 128;
            continue;
        }

        double dist = signedDistanceToPlane(pt, grid_cells_[it->second].plane_coeffs);
        uint8_t r, g, b;
        getColorForSignedDistance(dist, max_abs_height_, r, g, b);
        heatmap_cloud_->points[i].r = r;
        heatmap_cloud_->points[i].g = g;
        heatmap_cloud_->points[i].b = b;
    }
}

// ============ 工具函数 ============

Eigen::Vector4f MeasureWeldHeight::fitPlaneRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (!cloud || cloud->size() < 3) {
        return Eigen::Vector4f(0, 0, 1, 0);  // 默认水平面
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ransac_threshold_);
    ransac.setMaxIterations(200);
    ransac.computeModel();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    Eigen::Vector4f plane(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

    // 归一化
    Eigen::Vector3f normal = plane.head<3>();
    float norm = normal.norm();
    if (norm > 1e-10f) {
        plane /= norm;
    }

    return plane;
}

void MeasureWeldHeight::orientPlaneNormal(Eigen::Vector4f& plane,
    const Eigen::Vector3f& weld_centroid, const Eigen::Vector3f& base_centroid)
{
    // 使平面法向量指向"从壁面到焊缝"的方向
    Eigen::Vector3f direction = weld_centroid - base_centroid;
    Eigen::Vector3f normal = plane.head<3>();

    float dot = normal.dot(direction);
    if (dot < 0) {
        // 翻转法向量（同时翻转 D）
        plane = -plane;
    }
}

double MeasureWeldHeight::signedDistanceToPlane(const pcl::PointXYZ& point,
    const Eigen::Vector4f& plane)
{
    // plane = [A, B, C, D], 平面方程 Ax + By + Cz + D = 0
    // 有符号距离 = Ax + By + Cz + D (因为法向量已归一化)
    return plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3];
}

void MeasureWeldHeight::getColorForSignedDistance(double dist, double max_abs,
    uint8_t& r, uint8_t& g, uint8_t& b) const
{
    // 将距离归一化到 [-1, 1]
    double norm = std::max(-1.0, std::min(1.0, dist / max_abs));

    if (norm >= 0) {
        // 凸起: 绿(0.0) → 黄(0.5) → 红(1.0)
        if (norm < 0.5) {
            r = static_cast<uint8_t>(255 * (norm * 2));      // 0 → 255
            g = 255;
            b = 0;
        } else {
            r = 255;
            g = static_cast<uint8_t>(255 * (2 - norm * 2));  // 255 → 0
            b = 0;
        }
    } else {
        // 凹陷: 绿(0.0) → 青(-0.5) → 蓝(-1.0)
        double abs_norm = -norm;  // 取正，0 → 1
        if (abs_norm < 0.5) {
            // 绿(0,255,0) → 青(0,255,255)
            r = 0;
            g = 255;
            b = static_cast<uint8_t>(255 * (abs_norm * 2));
        } else {
            // 青(0,255,255) → 蓝(0,0,255)
            r = 0;
            g = static_cast<uint8_t>(255 * (2 - abs_norm * 2));
            b = 255;
        }
    }
}

bool MeasureWeldHeight::isPointValid(const pcl::PointXYZ& point) const
{
    return pcl::isFinite(point) &&
        !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z);
}

Eigen::Vector3f MeasureWeldHeight::computeCloudCentroid(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    return centroid.head<3>();
}

void MeasureWeldHeight::printDebug(const std::string& msg) const
{
    if (verbose_) {
        std::cout << "[MeasureWeldHeight] " << msg << std::endl;
    }
}
