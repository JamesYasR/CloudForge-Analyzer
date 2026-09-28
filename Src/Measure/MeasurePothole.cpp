#include "Measure/MeasurePothole.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <omp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <sstream>

// ============================================================
// 私有工具: 最小二乘椭圆拟合(Halir-Flusser 数值稳定版)
// 输入为展开面坐标 (x=轴向, y=周向弧长), 输出中心/长短半轴/旋转角
// ============================================================
namespace {

bool fitEllipseLSQ(const std::vector<Eigen::Vector2d>& pts,
                   Eigen::Vector2d& center, double& semi_major,
                   double& semi_minor, double& angle_rad)
{
    const int n = static_cast<int>(pts.size());
    if (n < 8) return false;

    // 1. 坐标归一化(提升数值稳定性)
    Eigen::Vector2d mu = Eigen::Vector2d::Zero();
    for (const auto& p : pts) mu += p;
    mu /= n;
    double scale = 0.0;
    for (const auto& p : pts) scale += (p - mu).squaredNorm();
    scale = std::sqrt(scale / n);
    if (scale < 1e-9) return false;

    Eigen::MatrixXd D1(n, 3), D2(n, 3);
    for (int i = 0; i < n; ++i) {
        const double x = (pts[i].x() - mu.x()) / scale;
        const double y = (pts[i].y() - mu.y()) / scale;
        D1(i, 0) = x * x;  D1(i, 1) = x * y;  D1(i, 2) = y * y;
        D2(i, 0) = x;      D2(i, 1) = y;      D2(i, 2) = 1.0;
    }
    const Eigen::Matrix3d S11 = D1.transpose() * D1;
    const Eigen::Matrix3d S12 = D1.transpose() * D2;
    const Eigen::Matrix3d S22 = D2.transpose() * D2;

    // 2. 约束矩阵的逆: 4AC - B^2 = 1 对应 (A,B,C) 的约束
    Eigen::Matrix3d C1;
    C1 << 0.0, 0.0, 0.5,
          0.0, -1.0, 0.0,
          0.5, 0.0, 0.0;
    const Eigen::Matrix3d S22inv = S22.inverse();

    // 3. 分块求解, 缩减为 3x3 广义特征问题
    const Eigen::Matrix3d M = C1 * (S11 - S12 * S22inv * S12.transpose());

    Eigen::EigenSolver<Eigen::Matrix3d> es(M);
    if (es.info() != Eigen::Success) return false;

    bool found = false;

    for (int k = 0; k < 3; ++k) {
        Eigen::Vector3d v1 = es.eigenvectors().col(k).real();
        if (v1.norm() < 1e-12) continue;

        // 约束定标: 缩放特征向量使 4AC - B^2 = 1 (椭圆判别式为正)
        const double disc = 4.0 * v1(0) * v1(2) - v1(1) * v1(1);
        if (disc <= 1e-14) continue;
        v1 /= std::sqrt(disc);
        const Eigen::Vector3d a2 = -S22inv * S12.transpose() * v1;

        // 反归一化基量: x = scale * x' + mu; 常数项 = 1 + 平移部分
        // (F=+1 约化约定下特征向量符号任意, 真实椭圆可能对应 s=±1 之一)
        const double dx = mu.x(), dy = mu.y(), a = scale;
        const double shift = (v1(0) * dx * dx + v1(1) * dx * dy + v1(2) * dy * dy) / (a * a)
            - (a2(0) * dx + a2(1) * dy) / a;

        for (int sgn = 0; sgn < 2; ++sgn) {
            const double sg = sgn == 0 ? 1.0 : -1.0;
            double A = sg * v1(0) / (a * a);
            double B = sg * v1(1) / (a * a);
            double C = sg * v1(2) / (a * a);
            double D = sg * ((-2.0 * v1(0) * dx - v1(1) * dy) / (a * a) + a2(0) / a);
            double E = sg * ((-2.0 * v1(2) * dy - v1(1) * dx) / (a * a) + a2(1) / a);
            double F = 1.0 + sg * shift;

            // 符号归一化到二次项正定(A>0), 同一曲线乘-1
            if (A < 0.0) {
                A = -A; B = -B; C = -C; D = -D; E = -E; F = -F;
            }
            if (4.0 * A * C - B * B <= 1e-12) continue;

            Eigen::Matrix2d Mq;
            Mq << 2.0 * A, B, B, 2.0 * C;
            Eigen::Vector2d rhs(-D, -E);
            const Eigen::Vector2d c0 = Mq.ldlt().solve(rhs);

            // 平移后的常数项: F0 = F + (D*x0 + E*y0)/2, 需为负才有实椭圆
            const double F0 = F + 0.5 * (D * c0.x() + E * c0.y());
            if (F0 >= -1e-12) continue;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(Mq);
            const double l1 = es2.eigenvalues()(0); // 小特征值 -> 长半轴
            const double l2 = es2.eigenvalues()(1); // 大特征值 -> 短半轴
            if (l1 <= 1e-12 || l2 <= 1e-12) continue;

            const double sm1 = std::sqrt(-F0 / l1);
            const double sm2 = std::sqrt(-F0 / l2);
            if (!(sm1 > 0.0) || !(sm2 > 0.0) || !std::isfinite(sm1) || !std::isfinite(sm2)) continue;

            center = c0;
            semi_major = sm1;
            semi_minor = sm2;
            // 主轴方向 = 0.5*atan2(B, A-C) + 90°(特征向量与半轴的配对关系)
            angle_rad = 0.5 * std::atan2(B, A - C) + 1.57079632679489661923;
            found = true;
            break;
        }
        if (found) break;
    }
    return found;
}

// 展开域点集凸包(Andrew 单调链). 返回逆时针顶点(不含共线中间点).
std::vector<Eigen::Vector2d> convexHull2D(const std::vector<double>& xs,
                                          const std::vector<double>& ys)
{
    const int n = static_cast<int>(std::min(xs.size(), ys.size()));
    std::vector<Eigen::Vector2d> p(n);
    for (int i = 0; i < n; ++i) p[i] = Eigen::Vector2d(xs[i], ys[i]);
    std::sort(p.begin(), p.end(), [](const Eigen::Vector2d& l, const Eigen::Vector2d& r) {
        return (l.x() < r.x()) || (l.x() == r.x() && l.y() < r.y());
    });
    p.erase(std::unique(p.begin(), p.end(),
                        [](const Eigen::Vector2d& l, const Eigen::Vector2d& r) {
                            return l.x() == r.x() && l.y() == r.y();
                        }), p.end());
    const int m = static_cast<int>(p.size());
    if (m < 3) return p;
    std::vector<Eigen::Vector2d> h(static_cast<std::size_t>(2 * m));
    int k = 0;
    auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& l,
                    const Eigen::Vector2d& r) {
        return (l.x() - o.x()) * (r.y() - o.y()) - (l.y() - o.y()) * (r.x() - o.x());
    };
    for (int i = 0; i < m; ++i) {          // 下凸包
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0.0) --k;
        h[k++] = p[i];
    }
    for (int i = m - 2, t = k + 1; i >= 0; --i) {  // 上凸包
        while (k >= t && cross(h[k - 2], h[k - 1], p[i]) <= 0.0) --k;
        h[k++] = p[i];
    }
    h.resize(static_cast<std::size_t>(std::max(0, k - 1)));
    return h;
}

} // namespace

// ============================================================
// 构造/析构
// ============================================================
MeasurePothole::MeasurePothole()
    : input_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , distance_threshold_(0.0)
    , cluster_tolerance_(0.0)
    , min_cluster_size_(200)
    , verbose_(true)
    , robust_sigma_(0.0)
    , effective_threshold_(0.0)
    , axis_point_(Eigen::Vector3f::Zero())
    , axis_direction_(Eigen::Vector3f::UnitZ())
    , t1_(Eigen::Vector3f::UnitX())
    , t2_(Eigen::Vector3f::UnitY())
    , design_radius_(0.0)
    , pit_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , heatmap_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , ellipse_major_(0.0)
    , ellipse_minor_(0.0)
    , ellipse_major_raw_(0.0)
    , ellipse_minor_raw_(0.0)
    , pit_mean_depth_(0.0)
    , cluster_count_(0)
    , deepest_index_(-1)
{
}

MeasurePothole::~MeasurePothole() = default;

// ============================================================
// 参数设置
// ============================================================
void MeasurePothole::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud && !cloud->empty()) {
        input_cloud_ = cloud;
        printDebugInfo("点云设置完成，点数: " + std::to_string(input_cloud_->size()));
    }
    else {
        printDebugInfo("错误: 点云为空或无效");
    }
}

void MeasurePothole::setCylinder(const pcl::ModelCoefficients::Ptr& cyl)
{
    if (!cyl || cyl->values.size() < 7) {
        printDebugInfo("错误: 圆柱结果无效(需要7个系数)");
        return;
    }
    if (!(cyl->values[6] > 0.0f) || !std::isfinite(cyl->values[6])) {
        printDebugInfo("错误: 圆柱半径无效");
        return;
    }
    cylinder_ = cyl;
    axis_point_ = Eigen::Vector3f(cyl->values[0], cyl->values[1], cyl->values[2]);
    axis_direction_ = Eigen::Vector3f(cyl->values[3], cyl->values[4], cyl->values[5]);
    if (axis_direction_.norm() < 1e-6f) {
        printDebugInfo("错误: 圆柱轴向为零向量");
        cylinder_.reset();
        return;
    }
    axis_direction_.normalize();
    design_radius_ = static_cast<double>(cyl->values[6]);

    // 垂直于轴的正交框架(展开面用)
    Eigen::Vector3f a = Eigen::Vector3f::UnitX();
    if (std::abs(a.dot(axis_direction_)) > 0.9f) a = Eigen::Vector3f::UnitY();
    t1_ = a - axis_direction_ * a.dot(axis_direction_);
    t1_.normalize();
    t2_ = axis_direction_.cross(t1_);

    printDebugInfo("圆柱结果设置: 半径 " + std::to_string(design_radius_) + " mm");
}

void MeasurePothole::setDistanceThreshold(double thr)
{
    if (thr >= 0.0) {
        distance_threshold_ = thr;
    }
    else {
        printDebugInfo("错误: 距离阈值必须非负(0=自动)");
    }
}

void MeasurePothole::setClusterTolerance(double tol)
{
    if (tol >= 0.0) {
        cluster_tolerance_ = tol;
    }
    else {
        printDebugInfo("错误: 聚类容差必须非负(0=自动)");
    }
}

void MeasurePothole::setMinClusterSize(int n)
{
    if (n > 0) {
        min_cluster_size_ = n;
    }
    else {
        printDebugInfo("错误: 最小点群点数必须为正");
    }
}

void MeasurePothole::setVerbose(bool verbose)
{
    verbose_ = verbose;
}

// ============================================================
// 步骤1: 全量点带符号径向残差 + 最深点
// ============================================================
bool MeasurePothole::computeResiduals()
{
    if (!cylinder_) {
        printDebugInfo("错误: 未设置圆柱结果");
        return false;
    }
    const int n = static_cast<int>(input_cloud_->size());
    residual_map_.assign(n, 0.0);

    const Eigen::Vector3f p0 = axis_point_;
    const Eigen::Vector3f d = axis_direction_;
    const double R = design_radius_;

#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        const auto& pt = (*input_cloud_)[i];
        if (!isPointValid(pt)) {
            residual_map_[i] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        const Eigen::Vector3f v(pt.x, pt.y, pt.z);
        const Eigen::Vector3f rad = v - p0 - d * (v - p0).dot(d);
        residual_map_[i] = static_cast<double>(rad.norm()) - R;
    }

    // 稳健统计: median + 1.4826*MAD (只统计有限值)
    std::vector<double> valid;
    valid.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (std::isfinite(residual_map_[i])) valid.push_back(residual_map_[i]);
    }
    if (valid.size() < 100) {
        printDebugInfo("错误: 有效点过少");
        return false;
    }
    const std::size_t nv = valid.size();
    std::nth_element(valid.begin(), valid.begin() + nv / 2, valid.end());
    const double med = valid[nv / 2];
    std::vector<double> absDev(nv);
    for (std::size_t i = 0; i < nv; ++i) absDev[i] = std::abs(valid[i] - med);
    std::nth_element(absDev.begin(), absDev.begin() + nv / 2, absDev.end());
    robust_sigma_ = 1.4826 * absDev[nv / 2];

    // 最深点(最小残差)
    deepest_index_ = -1;
    double min_e = 0.0;
    for (int i = 0; i < n; ++i) {
        const double e = residual_map_[i];
        if (!std::isfinite(e)) continue;
        if (deepest_index_ < 0 || e < min_e) {
            min_e = e;
            deepest_index_ = i;
        }
    }
    printDebugInfo("残差稳健 sigma = " + std::to_string(robust_sigma_) + " mm");
    return deepest_index_ >= 0;
}

// ============================================================
// 步骤1.5: 形面趋势(系统性偏差)估计与扣除
// 真实壁面相对理想圆柱总有 mm 级平滑形面偏差(碗形/倾斜). 若其幅度与阈值相当,
// "低于阈值"的点会连成大片, 被误当成凹塘. 这里在展开域做稳健低阶拟合,
// 得到趋势幅度与"趋势/局部起伏"比; 按模式(自动/不扣除/强制)决定是否扣除.
// 注意: 最大距离/最深点仍以原始残差(到理想柱面)为准, 符合需求口径.
// ============================================================
void MeasurePothole::estimateFormTrend()
{
    trend_removed_ = false;
    trend_p2p_ = 0.0;
    trend_ratio_ = 0.0;
    const int n = static_cast<int>(input_cloud_->size());
    residual_work_.assign(n, 0.0);

    // 1. 展开坐标(全量)并记录补丁范围
    std::vector<double> a(n), s(n);
    double amin = std::numeric_limits<double>::max(), amax = -amin;
    double smin = amin, smax = -amin;
    for (int i = 0; i < n; ++i) {
        const auto& p = (*input_cloud_)[i];
        const Eigen::Vector3f v(p.x, p.y, p.z);
        const Eigen::Vector3f rel = v - axis_point_;
        a[i] = static_cast<double>(rel.dot(axis_direction_));
        s[i] = design_radius_ * std::atan2(static_cast<double>(rel.dot(t2_)),
                                           static_cast<double>(rel.dot(t1_)));
        amin = std::min(amin, a[i]); amax = std::max(amax, a[i]);
        smin = std::min(smin, s[i]); smax = std::max(smax, s[i]);
    }
    patch_a_min_ = amin; patch_a_max_ = amax;
    patch_s_min_ = smin; patch_s_max_ = smax;
    patch_area_ = std::max(1e-6, (amax - amin) * (smax - smin));

    // 2. 稳健低阶拟合(子采样 + 两轮截尾): 基函数 [1, a, s] 或 [1, a, s, a^2, s^2, a*s]
    const int terms = (trend_order_ >= 2) ? 6 : 3;
    const int stride = std::max(1, n / 200000);
    std::vector<int> idx;
    idx.reserve(n / stride + 1);
    for (int i = 0; i < n; i += stride) {
        if (std::isfinite(residual_map_[i])) idx.push_back(i);
    }
    if (idx.size() < 50) {
        residual_work_ = residual_map_;
        return;
    }
    auto basis = [&](int i, int k) -> double {
        const double x = a[i], y = s[i];
        switch (k) {
        case 0: return 1.0;
        case 1: return x;
        case 2: return y;
        case 3: return x * x;
        case 4: return y * y;
        default: return x * y;
        }
    };
    std::vector<double> coef(static_cast<std::size_t>(terms), 0.0);
    std::vector<char> keep(idx.size(), 1);
    for (int round = 0; round < 2; ++round) {
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(terms, terms);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(terms);
        for (std::size_t t = 0; t < idx.size(); ++t) {
            if (!keep[t]) continue;
            const int i = idx[t];
            const double e = residual_map_[i];
            for (int r = 0; r < terms; ++r) {
                const double br = basis(i, r);
                b(r) += br * e;
                for (int c = r; c < terms; ++c) M(r, c) += br * basis(i, c);
            }
        }
        for (int r = 0; r < terms; ++r)
            for (int c = 0; c < r; ++c) M(r, c) = M(c, r);
        const Eigen::VectorXd sol = M.ldlt().solve(b);
        if (!sol.allFinite()) break;
        for (int k = 0; k < terms; ++k) coef[k] = sol(k);
        // 截尾: 剔除 |残差-趋势| > 3*sigma 的点
        std::vector<double> dev(idx.size());
        for (std::size_t t = 0; t < idx.size(); ++t) {
            double tr = 0.0;
            for (int k = 0; k < terms; ++k) tr += coef[k] * basis(idx[t], k);
            dev[t] = residual_map_[idx[t]] - tr;
        }
        std::vector<double> absd = dev;
        std::nth_element(absd.begin(), absd.begin() + absd.size() / 2, absd.end());
        const double med = absd[absd.size() / 2];
        const double mad = [&] {
            std::vector<double> t2(absd.size());
            for (std::size_t q = 0; q < absd.size(); ++q) t2[q] = std::abs(absd[q] - med);
            std::nth_element(t2.begin(), t2.begin() + t2.size() / 2, t2.end());
            return t2[t2.size() / 2];
        }();
        const double lim = 3.0 * 1.4826 * std::max(mad, 1e-9);
        for (std::size_t t = 0; t < idx.size(); ++t) keep[t] = (std::abs(dev[t] - med) <= lim) ? 1 : 0;
    }

    // 3. 趋势幅度/局部起伏 与 决策
    double tmin = std::numeric_limits<double>::max(), tmax = -tmin;
    for (std::size_t t = 0; t < idx.size(); ++t) {
        double tr = 0.0;
        for (int k = 0; k < terms; ++k) tr += coef[k] * basis(idx[t], k);
        tmin = std::min(tmin, tr); tmax = std::max(tmax, tr);
    }
    trend_p2p_ = (idx.empty() ? 0.0 : (tmax - tmin));

    std::vector<double> local(idx.size());
    for (std::size_t t = 0; t < idx.size(); ++t) {
        double tr = 0.0;
        for (int k = 0; k < terms; ++k) tr += coef[k] * basis(idx[t], k);
        local[t] = residual_map_[idx[t]] - tr;
    }
    std::nth_element(local.begin(), local.begin() + local.size() / 2, local.end());
    const double lmed = local[local.size() / 2];
    std::vector<double> ldev(local.size());
    for (std::size_t t = 0; t < local.size(); ++t) ldev[t] = std::abs(local[t] - lmed);
    std::nth_element(ldev.begin(), ldev.begin() + ldev.size() / 2, ldev.end());
    robust_sigma_work_ = 1.4826 * ldev[ldev.size() / 2];
    trend_ratio_ = trend_p2p_ / std::max(robust_sigma_work_, 1e-6);

    const bool significant = (trend_ratio_ > 2.0);
    const bool remove = (trend_mode_ == 2) || (trend_mode_ == 0 && significant);
    if (!remove) {
        residual_work_ = residual_map_;
        return;
    }
    // 4. 扣除趋势(全量)
#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        double tr = 0.0;
        for (int k = 0; k < terms; ++k) tr += coef[k] * basis(i, k);
        residual_work_[i] = std::isfinite(residual_map_[i]) ? (residual_map_[i] - tr)
                                                            : residual_map_[i];
    }
    trend_removed_ = true;
    // 扣除后重新统计局部 sigma
    {
        std::vector<double> w;
        w.reserve(n / stride + 1);
        for (int i = 0; i < n; i += stride)
            if (std::isfinite(residual_work_[i])) w.push_back(residual_work_[i]);
        if (!w.empty()) {
            std::nth_element(w.begin(), w.begin() + w.size() / 2, w.end());
            const double m = w[w.size() / 2];
            for (auto& v : w) v = std::abs(v - m);
            std::nth_element(w.begin(), w.begin() + w.size() / 2, w.end());
            robust_sigma_work_ = 1.4826 * w[w.size() / 2];
        }
    }
    printDebugInfo("形面趋势: p2p=" + std::to_string(trend_p2p_) + " mm, 局部sigma="
        + std::to_string(robust_sigma_work_) + " mm, 比值=" + std::to_string(trend_ratio_)
        + (trend_removed_ ? " -> 已扣除" : " -> 未扣除"));
}

// ============================================================
// 步骤1.8: 局部基准(口径B) —— 展开域栅格 + 大窗口稳健二阶拟合
// 见 docs/当前需新增功能/凹塘测量-局部基准架构设计.md §3:
//   1) 展开坐标(a,s)栅格化, cell ≈ max(2mm, 2x平均点距), 每格取中值 m(a,s)
//   2) 每个格在半径 W/2 的窗口内做「两轮截尾」最小二乘二阶拟合
//        b = c0 + c1*a + c2*s + c3*a^2 + c4*s^2 + c5*a*s  (窗口内以格中心为原点)
//   3) 局部凹陷 d = b - m (正 = 比周围表面低)
//   4) 距补丁边界 < W/2 的格不参与判定
// 口径A 的 residual_map_ 不被修改, 仅新增 lb_* 结果.
// ============================================================
bool MeasurePothole::computeLocalBaseline()
{
    local_ok_ = false;
    const int n = static_cast<int>(input_cloud_->size());
    local_dent_.assign(n, std::numeric_limits<double>::quiet_NaN());
    lb_cell_of_.assign(n, -1);
    local_max_depth_ = 0.0;
    local_mean_depth_ = 0.0;
    baseline_offset_p2p_ = 0.0;
    boundary_excluded_fraction_ = 0.0;
    local_deepest_point_ = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

    lb_cell_ = 0.0;
    if (!use_local_baseline_) {
        printDebugInfo("局部基准(口径B): 已关闭, 按口径A(到理想柱面距离)判定");
        return false;
    }
    const auto tStart = std::chrono::steady_clock::now();

    // ---- 1. 展开坐标(a, s) ----
    std::vector<double> ea(n), es(n);
    std::vector<char> ok(n, 0);
    double amin = patch_a_min_, amax = patch_a_max_;
    double smin = patch_s_min_, smax = patch_s_max_;
    const bool patchOk = std::isfinite(amin) && std::isfinite(amax) && std::isfinite(smin)
        && std::isfinite(smax) && (amax > amin) && (smax > smin);
    if (!patchOk) {
        amin = smin = std::numeric_limits<double>::max();
        amax = smax = -amin;
    }
    int nvalid = 0;
    for (int i = 0; i < n; ++i) {
        const auto& p = (*input_cloud_)[i];
        if (!isPointValid(p) || !std::isfinite(residual_map_[i])) continue;
        const Eigen::Vector3f v(p.x, p.y, p.z);
        const Eigen::Vector3f rel = v - axis_point_;
        const double a = static_cast<double>(rel.dot(axis_direction_));
        const double s = design_radius_ * std::atan2(static_cast<double>(rel.dot(t2_)),
                                                     static_cast<double>(rel.dot(t1_)));
        if (!std::isfinite(a) || !std::isfinite(s)) continue;
        ea[i] = a; es[i] = s; ok[i] = 1; ++nvalid;
        if (!patchOk) {
            amin = std::min(amin, a); amax = std::max(amax, a);
            smin = std::min(smin, s); smax = std::max(smax, s);
        }
    }
    if (nvalid < 100 || !(amax > amin) || !(smax > smin)) {
        printDebugInfo("局部基准(口径B): 有效点/补丁范围不足, 无法建立");
        return false;
    }

    // ---- 2. 栅格: cell = clamp(W/90, max(2x平均点距, 0.5mm), 3mm), 每格中值 m ----
    // 规格要求: 小尺寸凹坑(φ10~30mm)必须在栅格上有足够格数(否则轮廓/椭圆不可靠).
    //   旧规则 max(2mm, 2x点距) -> 恒为 2mm, φ11mm 的坑只有约 5 格, 椭圆失真;
    //   新规则把格边长与窗口 W 绑定: W=90 -> 1.0mm, W=200 -> 2.22mm(上限 3mm).
    // 为什么取 W/90 而不是规格草案的 W/60:
    //   本样本实测 W/60(W=90 -> 1.5mm)时 φ11mm 的坑只有 3 个判定格(145 点),
    //   轮廓点/点数都不足以拟合椭圆, 该坑只能报"无法拟合";
    //   取 W/90(W=90 -> 1.0mm)时同一坑有十几个格, 轮廓/椭圆可用.
    //   Python 独立标定也正是用 W=90 + 1.0mm 格取得 -0.02~-0.11mm 的深度误差.
    // 上下限: 上限 3mm 保证大窗口时格数/耗时可控; 下限 max(2x点距, 0.5mm) 保证每格有点.
    const double patchArea = (amax - amin) * (smax - smin);
    const double spacing = std::sqrt(std::max(patchArea, 1e-6) / static_cast<double>(nvalid));
    const double cellMin = std::max(0.5, 2.0 * spacing);
    double cell = std::min(std::max(local_window_ / 90.0, cellMin), 3.0);
    const double halfW = 0.5 * local_window_;
    int na = 1, ns = 1;
    // 硬上限: 格数上限 4e6 -> 2.4e7(见报告"格数上限调整"). 1mm 格/300x338mm 补丁约 10 万格,
    // 远低于该上限, 不会触发放大; 极端大补丁(3m x 1m)才会按 1.5 倍逐级放大格边长.
    const double kMaxCells = 2.4e7;
    for (int guard = 0; guard < 40; ++guard) {
        na = std::max(3, static_cast<int>(std::ceil((amax - amin) / cell)) + 1);
        ns = std::max(3, static_cast<int>(std::ceil((smax - smin) / cell)) + 1);
        const double nc = static_cast<double>(na) * ns;
        if (nc <= kMaxCells) break;
        cell *= 1.5;   // 极端大补丁: 放大格边长, 控制内存/耗时
    }
    lb_cell_ = cell;    // 记录本次实际格边长(报告"本次使用参数"用)
    const std::size_t nc = static_cast<std::size_t>(na) * ns;
    auto cellOf = [&](double a, double s) -> int {
        const int ia = std::min(na - 1, std::max(0, static_cast<int>((a - amin) / cell)));
        const int is = std::min(ns - 1, std::max(0, static_cast<int>((s - smin) / cell)));
        return ia * ns + is;
    };

    // 计数排序(点->格), 避免每格一个 vector 的分配
    std::vector<int> counts(nc, 0);
    for (int i = 0; i < n; ++i) {
        if (!ok[i]) continue;
        const int c = cellOf(ea[i], es[i]);
        lb_cell_of_[i] = c;
        ++counts[c];
    }
    std::vector<int> offset(nc + 1, 0);
    for (std::size_t c = 0; c < nc; ++c) offset[c + 1] = offset[c] + counts[c];
    std::vector<double> cellData(static_cast<std::size_t>(nvalid));
    {
        std::vector<int> cursor(offset.begin(), offset.end() - 1);
        for (int i = 0; i < n; ++i) {
            if (!ok[i]) continue;
            cellData[cursor[lb_cell_of_[i]]++] = residual_map_[i];
        }
    }
    lb_med_.assign(nc, std::numeric_limits<double>::quiet_NaN());
    int nCellValid = 0;
    for (std::size_t c = 0; c < nc; ++c) {
        const int lo = offset[c], cnt = offset[c + 1] - offset[c];
        if (cnt <= 0) continue;
        double* beg = cellData.data() + lo;
        std::nth_element(beg, beg + cnt / 2, beg + cnt);
        lb_med_[c] = beg[cnt / 2];
        ++nCellValid;
    }

    // ---- 3. 边界排除: 距补丁边界 < W/2 的格不参与判定 ----
    std::vector<char> excl(nc, 0);
    std::size_t nExcl = 0;
    if (boundary_exclude_) {
        for (int ia = 0; ia < na; ++ia) {
            const double ac = amin + (ia + 0.5) * cell;
            for (int is = 0; is < ns; ++is) {
                const double sc = smin + (is + 0.5) * cell;
                const double dEdge = std::min(std::min(ac - amin, amax - ac),
                                              std::min(sc - smin, smax - sc));
                if (dEdge < halfW) {
                    excl[static_cast<std::size_t>(ia) * ns + is] = 1;
                    ++nExcl;
                }
            }
        }
    }
    boundary_excluded_fraction_ = static_cast<double>(nExcl) / static_cast<double>(nc);
    const auto tGrid = std::chrono::steady_clock::now();

    // ---- 4. 大窗口两轮截尾二阶拟合 -> b ----
    // 拟合节点间隔: 格数很多时按 stride 稀疏取节点, 节点间双线性插值(大补丁加速)
    int nodeStride = 1;
    while (nCellValid / (nodeStride * nodeStride) > 60000) ++nodeStride;
    const int rad = std::max(1, static_cast<int>(std::ceil(halfW / cell)));
    const double halfW2 = halfW * halfW;
    const std::size_t maxSamples = 400;

    lb_base_.assign(nc, std::numeric_limits<double>::quiet_NaN());
    const int stride = std::max(1, nodeStride);
    int nFit = 0, nFallback = 0;

    // 每个节点独立: 线程内复用暂存缓冲(避免逐节点堆分配), 节点间无写冲突
    // (本函数运行在 worker 线程, 不触碰任何 GUI 对象)
#pragma omp parallel
    {
        std::vector<double> sa, ss, sv, A(6), resid;
        std::vector<char> keep;
        Eigen::MatrixXd M(6, 6);
        Eigen::VectorXd rhs(6);
        int fitCnt = 0, fbCnt = 0;

        auto fitWindow = [&](double ac, double sc, double& bOut) -> bool {
            const std::size_t m0 = sa.size();
            if (m0 < 8) return false;
            // 样本过多时等间隔抽稀(二阶拟合无需全部格)
            std::size_t step = 1;
            if (m0 > maxSamples) step = (m0 + maxSamples - 1) / maxSamples;
            const std::size_t m = (m0 + step - 1) / step;
            int terms = (m >= 24) ? 6 : 3;
            for (int attempt = 0; attempt < 2; ++attempt) {
                keep.assign(m0, 1);
                bool solved = false;
                for (int round = 0; round < 2; ++round) {
                    M.topLeftCorner(terms, terms).setZero();
                    rhs.head(terms).setZero();
                    for (std::size_t t = 0; t < m0; t += step) {
                        if (!keep[t]) continue;
                        const double x = sa[t] - ac, y = ss[t] - sc, v = sv[t];
                        const double bs[6] = { 1.0, x, y, x * x, y * y, x * y };
                        for (int r = 0; r < terms; ++r) {
                            rhs(r) += bs[r] * v;
                            for (int c2 = r; c2 < terms; ++c2) M(r, c2) += bs[r] * bs[c2];
                        }
                    }
                    for (int r = 0; r < terms; ++r)
                        for (int c2 = 0; c2 < r; ++c2) M(r, c2) = M(c2, r);
                    Eigen::VectorXd sol;
                    if (terms == 6) {
                        sol = M.ldlt().solve(rhs);
                    }
                    else {   // 降阶(样本过少/病态): 固定 3x3
                        const Eigen::Matrix3d M3 = M.topLeftCorner(3, 3);
                        const Eigen::Vector3d r3 = rhs.head(3);
                        sol = M3.ldlt().solve(r3);
                    }
                    if (!sol.allFinite()) break;
                    for (int k = 0; k < terms; ++k) A[k] = sol(k);
                    solved = true;
                    // 截尾: 剔除 |m - b| 偏离中位数 > 3*sigma 的格(坑/凸起/空白边界)
                    resid.clear();
                    if (round == 0) {
                        for (std::size_t t = 0; t < m0; t += step) {
                            if (!keep[t]) continue;
                            const double x = sa[t] - ac, y = ss[t] - sc;
                            double tr = 0.0;
                            const double bs[6] = { 1.0, x, y, x * x, y * y, x * y };
                            for (int k = 0; k < terms; ++k) tr += A[k] * bs[k];
                            resid.push_back(sv[t] - tr);
                        }
                        if (resid.size() < static_cast<std::size_t>(terms) + 2) break;
                        std::vector<double> tmp = resid;
                        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
                        const double med = tmp[tmp.size() / 2];
                        for (double& v : tmp) v = std::abs(v - med);
                        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
                        const double sigma = std::max(1.4826 * tmp[tmp.size() / 2], 1e-6);
                        const double lim = 3.0 * sigma;
                        std::size_t q = 0;
                        int kept = 0;
                        for (std::size_t t = 0; t < m0; t += step) {
                            if (!keep[t]) continue;
                            keep[t] = (std::abs(resid[q] - med) <= lim) ? 1 : 0;
                            kept += keep[t];
                            ++q;
                        }
                        // 剔除过狠(<10% 样本或不足阶数+2)则退回不剔除, 避免病态拟合
                        if (kept < static_cast<int>(std::max<std::size_t>(terms + 2, m / 10))) {
                            keep.assign(m0, 1);
                            break;
                        }
                    }
                }
                if (solved) {
                    bOut = A[0];   // 窗口以格中心为原点, 常数项即该格基准值
                    return true;
                }
                if (terms == 3) break;
                terms = 3;         // 二阶退化(样本过少/病态) -> 一阶
            }
            // 最后退路: 窗口内中值作为基准
            if (terms == 3) {
                resid.clear();
                for (std::size_t t = 0; t < m0; t += step) resid.push_back(sv[t]);
                std::nth_element(resid.begin(), resid.begin() + resid.size() / 2, resid.end());
                bOut = resid[resid.size() / 2];
                ++fbCnt;
                return true;
            }
            return false;
        };

#pragma omp for schedule(dynamic, 32)
        for (int ia = 0; ia < na; ia += stride) {
            for (int is = 0; is < ns; is += stride) {
                const std::size_t c = static_cast<std::size_t>(ia) * ns + is;
                if (!std::isfinite(lb_med_[c])) continue;
                sa.clear(); ss.clear(); sv.clear();
                for (int da = -rad; da <= rad; ++da) {
                    const int ja = ia + da;
                    if (ja < 0 || ja >= na) continue;
                    for (int ds = -rad; ds <= rad; ++ds) {
                        const int js = is + ds;
                        if (js < 0 || js >= ns) continue;
                        const double da2 = da * cell, ds2 = ds * cell;
                        if (da2 * da2 + ds2 * ds2 > halfW2) continue;
                        const std::size_t cn = static_cast<std::size_t>(ja) * ns + js;
                        if (!std::isfinite(lb_med_[cn])) continue;
                        sa.push_back(amin + (ja + 0.5) * cell);
                        ss.push_back(smin + (js + 0.5) * cell);
                        sv.push_back(lb_med_[cn]);
                    }
                }
                double b = 0.0;
                if (fitWindow(amin + (ia + 0.5) * cell, smin + (is + 0.5) * cell, b)) {
                    lb_base_[c] = b;      // 各节点写各自的格, 无竞争
                    ++fitCnt;
                }
            }
        }
#pragma omp atomic
        nFit += fitCnt;
#pragma omp atomic
        nFallback += fbCnt;
    }
    if (nFit <= 0) {
        printDebugInfo("局部基准(口径B): 窗口拟合失败(数据过稀)");
        return false;
    }

    // ---- 5. 节点->全格(必要时双线性插值) + 局部凹陷 d = b - m ----
    // 格数可达 1e5~1e7 量级(1mm 格), 这里按平坦下标并行(各格互不依赖),
    // 极值/计数用 reduction, 保证与串行结果一致.
    lb_dent_.assign(nc, std::numeric_limits<double>::quiet_NaN());
    lb_dent_all_.assign(nc, std::numeric_limits<double>::quiet_NaN());
    lb_judge_.assign(nc, 0);
    double dmax = -std::numeric_limits<double>::max();
    double bmin = std::numeric_limits<double>::max(), bmax = -bmin;
    long long deepestCell = -1;
    int nJudge = 0;
    const int nodeStrideL = nodeStride;   // 供 OpenMP 默认(shared)捕获
    const bool boundaryExclL = boundary_exclude_;
#pragma omp parallel for schedule(static) reduction(+:nJudge) reduction(max:dmax) \
    reduction(min:bmin) reduction(max:bmax)
    for (long long flat = 0; flat < static_cast<long long>(nc); ++flat) {
        const int ia = static_cast<int>(flat / ns);
        const int is = static_cast<int>(flat % ns);
        const std::size_t c = static_cast<std::size_t>(flat);
        if (!std::isfinite(lb_med_[c])) continue;
        double base = std::numeric_limits<double>::quiet_NaN();
        if (nodeStrideL == 1) {
            base = lb_base_[c];
        }
        else {
            const int ia0 = (ia / nodeStrideL) * nodeStrideL;
            const int ia1 = std::min(ia0 + nodeStrideL, na - 1);
            const int is0 = (is / nodeStrideL) * nodeStrideL;
            const int is1 = std::min(is0 + nodeStrideL, ns - 1);
            const double wa = (ia1 > ia0) ? static_cast<double>(ia - ia0) / (ia1 - ia0) : 0.0;
            const double ws = (is1 > is0) ? static_cast<double>(is - is0) / (is1 - is0) : 0.0;
            const int ja[2] = { ia0, ia1 };
            const int js[2] = { is0, is1 };
            const double w[4] = { (1 - wa) * (1 - ws), wa * (1 - ws), (1 - wa) * ws, wa * ws };
            double sum = 0.0, wsum = 0.0;
            for (int p = 0; p < 2; ++p) {
                for (int q = 0; q < 2; ++q) {
                    const std::size_t cn = static_cast<std::size_t>(ja[p]) * ns + js[q];
                    const double ww = w[p * 2 + q];
                    if (ww <= 0.0 || !std::isfinite(lb_base_[cn])) continue;
                    sum += ww * lb_base_[cn];
                    wsum += ww;
                }
            }
            if (wsum > 1e-9) base = sum / wsum;
        }
        if (!std::isfinite(base)) continue;
        lb_dent_all_[c] = base - lb_med_[c];      // 不受边界排除影响(逐坑深度统计用)
        lb_dent_[c] = base - lb_med_[c];
        if (boundaryExclL && excl[c]) continue;   // 边界区不参与判定
        lb_judge_[c] = 1;
        ++nJudge;
        if (dmax < lb_dent_[c]) dmax = lb_dent_[c];
        bmin = std::min(bmin, base);
        bmax = std::max(bmax, base);
    }
    if (nJudge <= 0) {
        printDebugInfo("局部基准(口径B): 全部格被边界排除, 无判定区");
        return false;
    }
    // reduction 不携带下标, 再扫一遍判定区找最深格(只比较, 很快)
    for (long long flat = 0; flat < static_cast<long long>(nc); ++flat) {
        const std::size_t c = static_cast<std::size_t>(flat);
        if (lb_judge_[c] && lb_dent_[c] == dmax) { deepestCell = flat; break; }
    }
    local_max_depth_ = std::max(0.0, dmax);
    baseline_offset_p2p_ = (bmax >= bmin) ? (bmax - bmin) : 0.0;

    // ---- 6. 每点局部凹陷(热力图用) 与 局部口径最深点 ----
#pragma omp parallel for schedule(static)
    for (int i = 0; i < n; ++i) {
        const int c = lb_cell_of_[i];
        if (c < 0 || !std::isfinite(lb_base_[c]) || !std::isfinite(residual_map_[i])) continue;
        local_dent_[i] = lb_base_[c] - residual_map_[i];
    }
    if (deepestCell >= 0 && deepestCell < static_cast<long long>(nc)) {
        const double mTarget = lb_med_[static_cast<std::size_t>(deepestCell)];
        double bestDev = std::numeric_limits<double>::max();
        for (int i = 0; i < n; ++i) {
            if (lb_cell_of_[i] != static_cast<int>(deepestCell) || !std::isfinite(residual_map_[i]))
                continue;
            const double dev = std::abs(residual_map_[i] - mTarget);
            if (dev < bestDev) {
                bestDev = dev;
                local_deepest_point_ = (*input_cloud_)[i];
            }
        }
    }

    local_ok_ = true;
    const auto tEnd = std::chrono::steady_clock::now();
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "局部基准(口径B): 栅格 " << na << "x" << ns << " cell=" << cell
           << "mm, 窗口 W=" << local_window_ << "mm, 阈值 T_local=" << local_threshold_
           << "mm, 拟合节点 " << nFit << "(stride=" << nodeStride << ", 中值退路 " << nFallback
           << "), 判定格 " << nJudge << "/" << nc
           << ", 边界排除 " << boundary_excluded_fraction_ * 100.0 << "%"
           << ", 最大局部下凹 " << local_max_depth_ << "mm"
           << ", 基准偏移峰峰 " << baseline_offset_p2p_ << "mm"
           << ", 耗时 " << std::chrono::duration<double, std::milli>(tGrid - tStart).count()
           << "ms(栅格)+" << std::chrono::duration<double, std::milli>(tEnd - tGrid).count()
           << "ms(拟合)";
        printDebugInfo(ss.str());
    }
    return true;
}

// ============================================================
// 步骤2: 阈值提取凹塘候选 + 欧氏聚类取最大簇
// ============================================================
bool MeasurePothole::extractPitCluster()
{
    pit_cloud_->clear();
    cluster_count_ = 0;
    local_mean_depth_ = 0.0;
    pit_items_.clear();

    const bool useLocal = use_local_baseline_ && local_ok_;

    // 候选点:
    //   口径B(局部基准): 判定格内 局部凹陷 d = b - m > T_local
    //   口径A(到理想柱面): e < -thr
    pcl::PointCloud<pcl::PointXYZ>::Ptr cand(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> candToOrig;
    cand->points.reserve(input_cloud_->size() / 10);
    candToOrig.reserve(input_cloud_->size() / 10);

    if (useLocal) {
        // 阈值: 局部阈值(手动给定, 默认 0.5mm, 不再用稳健自动值)
        effective_threshold_ = local_threshold_;
        printDebugInfo("局部阈值 T_local = " + std::to_string(effective_threshold_)
            + " mm (口径B: 相对局部基准的下凹量)");
        for (int i = 0; i < static_cast<int>(input_cloud_->size()); ++i) {
            const double e = residual_map_[i];
            if (!std::isfinite(e)) continue;
            const int c = lb_cell_of_[i];
            if (c < 0 || !lb_judge_[c]) continue;               // 无效格/边界排除格不参与判定
            if (lb_dent_[c] > local_threshold_) {
                cand->points.push_back((*input_cloud_)[i]);
                candToOrig.push_back(i);
            }
        }
    }
    else {
        // 阈值: 手动值或稳健自动值(基准为"工作残差": 若已扣除形面趋势则以扣除后为准)
        if (distance_threshold_ > 0.0) {
            effective_threshold_ = distance_threshold_;
        }
        else {
            effective_threshold_ = std::max(3.0 * robust_sigma_work_, 0.8);
        }
        printDebugInfo("距离阈值 = " + std::to_string(effective_threshold_) + " mm"
            + (trend_removed_ ? " (已扣除形面趋势)" : ""));

        for (int i = 0; i < static_cast<int>(input_cloud_->size()); ++i) {
            const double e = residual_work_[i];
            if (std::isfinite(e) && e < -effective_threshold_) {
                cand->points.push_back((*input_cloud_)[i]);
                candToOrig.push_back(i);
            }
        }
    }
    cand->width = static_cast<std::uint32_t>(cand->points.size());
    cand->height = 1;
    cand->is_dense = true;
    printDebugInfo("候选点数 = " + std::to_string(cand->size()));
    if (cand->size() < static_cast<std::size_t>(min_cluster_size_)) {
        printDebugInfo("候选点不足，未检出凹塘点群");
        return false;
    }

    // 欧氏聚类
    const double tol = cluster_tolerance_ > 0.0 ? cluster_tolerance_ : 3.0;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cand);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(static_cast<int>(cand->size()));
    ec.setSearchMethod(tree);
    ec.setInputCloud(cand);
    ec.extract(clusters);
    cluster_count_ = static_cast<int>(clusters.size());
    printDebugInfo("聚类容差 = " + std::to_string(tol) + " mm, 簇数 = "
        + std::to_string(cluster_count_));
    if (clusters.empty()) {
        return false;
    }

    // 多凹坑: 每个满足 min_cluster_size 的簇都做成一个独立结果(四个坑 = 四个结果).
    // 按点数降序排序: 主坑(面积最大者)= 第一个, 同时写回原有的单坑字段以兼容旧调用方.
    std::vector<int> order(clusters.size());
    for (std::size_t i = 0; i < clusters.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int l, int r) {
        return clusters[l].indices.size() > clusters[r].indices.size();
    });

    pit_items_.clear();
    pit_items_.reserve(order.size());
    int idx = 0;
    for (int ci : order) {
        const pcl::PointIndices& cl = clusters[ci];
        if (cl.indices.size() < static_cast<std::size_t>(min_cluster_size_)) continue;
        PitItem item;
        if (!buildPitItem(++idx, cl, candToOrig, useLocal, item)) {
            --idx;            // 该簇连点群/坐标都不成立: 不计入
            continue;
        }
        pit_items_.push_back(std::move(item));
    }
    if (pit_items_.empty()) {
        printDebugInfo("满足最小点数的凹塘点群: 无");
        return false;
    }

    // 主坑(点数最多者)写回原有单坑语义的字段
    {
        const PitItem& mainPit = pit_items_.front();
        // pit_cloud_ 必须恢复成主坑点群: buildPitItem 是逐簇写的(循环结束后停在该簇)
        pit_cloud_->points.clear();
        for (int k : clusters[order[0]].indices) {
            pit_cloud_->points.push_back((*input_cloud_)[candToOrig[k]]);
        }
        pit_cloud_->width = static_cast<std::uint32_t>(pit_cloud_->points.size());
        pit_cloud_->height = 1;
        pit_cloud_->is_dense = true;
        ellipse_points_ = mainPit.ellipse_points;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "凹塘点群 = 共 " << pit_items_.size() << " 个, 主坑 " << mainPit.pit_points
           << " 点 (局部最大下凹 " << mainPit.local_max_depth
           << " mm, 长短轴 " << mainPit.major_axis << "x" << mainPit.minor_axis << " mm)";
        printDebugInfo(ss.str());
    }
    return true;
}

// ============================================================
// 步骤2b: 单个簇 -> PitItem(点群 -> 轮廓 -> 椭圆 -> 回投 -> 四道闸门 -> 数值统计)
// 说明: 复用椭圆拟合的成员状态(ellipse_* / pit_* 校验量), 因为它是逐簇重算的;
//       ellipse_points_ 始终保存"当前这一簇"的回投折线, 逐簇拷贝进 PitItem.
//       主坑(第一个)把这些状态保留下来, 与改造前的单坑字段语义一致.
// ============================================================
bool MeasurePothole::buildPitItem(int index, const pcl::PointIndices& cluster,
                                  const std::vector<int>& candToOrig, bool useLocal, PitItem& out)
{
    const int cnt = static_cast<int>(cluster.indices.size());
    if (cnt <= 0 || index <= 0) return false;

    // 1. 点群(3D) + 逐簇数值统计(局部口径 d / 口径A e)
    pcl::PointCloud<pcl::PointXYZ> clCloud;
    clCloud.points.reserve(cluster.indices.size());
    double sumDepth = 0.0, sumLocalDepth = 0.0, maxDepth = 0.0, maxLocal = 0.0;
    int cntLocal = 0;
    for (int k : cluster.indices) {
        const int orig = candToOrig[k];
        clCloud.points.push_back((*input_cloud_)[orig]);
        const double e = residual_map_[orig];
        if (std::isfinite(e)) {
            sumDepth += -e;
            maxDepth = std::max(maxDepth, -e);
        }
        // 逐坑深度统计用"格中值"而非逐点值: 逐点值含 0.35mm 噪声, 单个点的
        // 最大下凹会比坑的真实深度高出 1mm 以上(实测 2.60 vs 真实 1.5).
        // 格中值已把点噪声平均掉, 与 Python 独立标定口径一致.
        const int c = (orig >= 0 && orig < static_cast<int>(lb_cell_of_.size())) ? lb_cell_of_[orig] : -1;
        if (c >= 0 && static_cast<std::size_t>(c) < lb_dent_all_.size()
            && std::isfinite(lb_dent_all_[c])) {
            sumLocalDepth += lb_dent_all_[c];
            maxLocal = std::max(maxLocal, lb_dent_all_[c]);
            ++cntLocal;
        }
    }
    if (clCloud.points.empty()) return false;
    clCloud.width = static_cast<std::uint32_t>(clCloud.points.size());
    clCloud.height = 1;
    clCloud.is_dense = true;

    PitItem& item = out;
    item = PitItem();      // 复用调用方传入的对象, 先清空
    item.index = index;
    item.pit_points = cnt;
    item.judged_by_local = useLocal;
    item.global_max_depth = std::max(0.0, maxDepth);
    item.global_mean_depth = sumDepth / std::max(1, cnt);
    // 局部口径: 该簇的局部最大/平均下凹(正 = 比周围表面低). 对"格"取统计,
    // 因此先对格去重(一个格内有很多点, 直接按点求均值等于按点密度加权).
    (void)sumLocalDepth; (void)cntLocal;
    {
        std::vector<int> cellsOfPit;
        cellsOfPit.reserve(cluster.indices.size());
        for (int k : cluster.indices) {
            const int orig = candToOrig[k];
            const int c = (orig >= 0 && orig < static_cast<int>(lb_cell_of_.size())) ? lb_cell_of_[orig] : -1;
            if (c >= 0) cellsOfPit.push_back(c);
        }
        std::sort(cellsOfPit.begin(), cellsOfPit.end());
        cellsOfPit.erase(std::unique(cellsOfPit.begin(), cellsOfPit.end()), cellsOfPit.end());
        double sCell = 0.0, mCell = 0.0;
        int nCell = 0;
        for (int c : cellsOfPit) {
            if (static_cast<std::size_t>(c) >= lb_dent_all_.size()) continue;
            const double d = lb_dent_all_[c];
            if (!std::isfinite(d)) continue;
            sCell += d;
            mCell = std::max(mCell, d);
            ++nCell;
        }
        item.pit_cells = nCell;
        item.local_mean_depth = (nCell > 0) ? (sCell / nCell) : 0.0;
        item.local_max_depth = std::max(0.0, nCell > 0 ? mCell : maxLocal);
    }

    // 2. 展开域坐标 + 质心(3D 先求均值: 展开域 s 有 ±180 回绕, 3D 均值更稳)
    std::vector<double> pa(cnt), ps(cnt);
    Eigen::Vector3d sum3 = Eigen::Vector3d::Zero();
    double amin = std::numeric_limits<double>::max(), amax = -amin;
    double smin = amin, smax = -amin;
    for (int i = 0; i < cnt; ++i) {
        const auto& p = clCloud.points[i];
        const Eigen::Vector3f v(p.x, p.y, p.z);
        const Eigen::Vector3f rel = v - axis_point_;
        pa[i] = static_cast<double>(rel.dot(axis_direction_));
        ps[i] = design_radius_ * std::atan2(static_cast<double>(rel.dot(t2_)),
                                            static_cast<double>(rel.dot(t1_)));
        amin = std::min(amin, pa[i]); amax = std::max(amax, pa[i]);
        smin = std::min(smin, ps[i]); smax = std::max(smax, ps[i]);
        sum3 += Eigen::Vector3d(p.x, p.y, p.z);
    }
    const Eigen::Vector3d c3 = sum3 / static_cast<double>(cnt);
    {
        const Eigen::Vector3d rel = c3 - axis_point_.cast<double>();
        const Eigen::Vector3d adD = axis_direction_.cast<double>();
        item.centroid_a = rel.dot(adD);
        item.centroid_s = design_radius_ * std::atan2(rel.dot(t2_.cast<double>()),
                                                      rel.dot(t1_.cast<double>()));
    }
    item.centroid = pcl::PointXYZ(static_cast<float>(c3.x()), static_cast<float>(c3.y()),
                                  static_cast<float>(c3.z()));
    item.contour_span_a = amax - amin;
    item.contour_span_s = smax - smin;

    // 3. 簇内最深的那个点(局部口径优先; 用"距本簇最深格中值最近"的点, 与主口径一致)
    {
        int bestIdx = 0;
        double bestVal = -std::numeric_limits<double>::max();
        for (int i = 0; i < cnt; ++i) {
            const int orig = candToOrig[cluster.indices[i]];
            const double d = std::isfinite(local_dent_[orig]) ? local_dent_[orig]
                                                              : -residual_map_[orig];
            if (d > bestVal) { bestVal = d; bestIdx = i; }
        }
        item.deepest_point = clCloud.points[bestIdx];
    }

    // 4. 轮廓 + 椭圆 + 回投: 复用 fitEllipseOnContour()(对 pit_cloud_ 当前内容工作)
    pit_cloud_->points = clCloud.points;
    pit_cloud_->width = clCloud.width;
    pit_cloud_->height = 1;
    pit_cloud_->is_dense = true;
    const bool hasEllipse = fitEllipseOnContour();
    item.ellipse_ok = hasEllipse;
    if (hasEllipse) {
        item.major_axis = ellipse_major_;
        item.minor_axis = ellipse_minor_;
        item.major_axis_raw = ellipse_major_raw_;
        item.minor_axis_raw = ellipse_minor_raw_;
        item.aspect_ratio = ellipse_aspect_;
        item.ellipse_center_a = ellipse_cx_;
        item.ellipse_center_s = ellipse_cy_;
        item.ellipse_out_of_patch = ellipse_out_of_patch_;
        item.ellipse_points = ellipse_points_;
    }
    item.area_fraction = pit_area_fraction_;
    item.touch_boundary = pit_touch_boundary_;

    // 5. 四道闸门
    std::string reason;
    if (!hasEllipse) {
        reason = "点群过小或无法拟合成椭圆(非凹坑形状)";
        item.valid = false;
    }
    else {
        item.valid = validatePit(reason);
    }
    item.reject_reason = reason;

    // 6. 主坑: 保留逐簇状态供 PitResult 原有字段/椭圆可视化使用(椭圆点集已拷贝到 item)
    if (index == 1) {
        item.is_main = true;
        pit_mean_depth_ = item.global_mean_depth;
        local_mean_depth_ = item.local_mean_depth;
        ellipse_major_ = item.major_axis;
        ellipse_minor_ = item.minor_axis;
    }
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "  坑#" << item.index << ": " << item.pit_points << " 点/" << item.pit_cells << " 格"
           << ", 局部最大下凹 " << item.local_max_depth << " mm"
           << ", 局部平均 " << item.local_mean_depth << " mm"
           << ", 口径A 最大 " << item.global_max_depth << " mm"
           << ", 长短轴 " << item.major_axis << "x" << item.minor_axis << " mm"
           << ", 中心(a,s)=(" << item.ellipse_center_a << ", " << item.ellipse_center_s << ")"
           << ", 质心(a,s)=(" << item.centroid_a << ", " << item.centroid_s << ")"
           << ", 判定=" << (item.valid ? "有效凹塘" : ("不判定: " + item.reject_reason));
        printDebugInfo(ss.str());
    }
    return true;
}

// ============================================================
// 步骤3: 展开域边界提取 + 椭圆拟合 + 回投柱面
// ============================================================
bool MeasurePothole::fitEllipseOnContour()
{
    ellipse_points_.clear();
    ellipse_major_ = ellipse_minor_ = 0.0;
    ellipse_major_raw_ = ellipse_minor_raw_ = 0.0;
    pit_area_fraction_ = 0.0;
    pit_touch_boundary_ = false;
    ellipse_out_of_patch_ = false;
    ellipse_aspect_ = 0.0;
    const std::size_t np = pit_cloud_->size();
    if (np < 30) {   // 小尺寸凹坑(φ11mm)只有 1~2 百点, 门槛不能定得太高
        printDebugInfo("点群过小，跳过椭圆拟合");
        return false;
    }

    // 1. 展开坐标: a = 轴向, s = R*phi (周向弧长)
    std::vector<double> a(np), s(np);
    double amin = std::numeric_limits<double>::max(), amax = -amin;
    double smin = amin, smax = -amin;
    for (std::size_t i = 0; i < np; ++i) {
        const Eigen::Vector3f v((*pit_cloud_)[i].x, (*pit_cloud_)[i].y, (*pit_cloud_)[i].z);
        const Eigen::Vector3f rel = v - axis_point_;
        a[i] = static_cast<double>(rel.dot(axis_direction_));
        s[i] = design_radius_ * std::atan2(static_cast<double>(rel.dot(t2_)),
                                           static_cast<double>(rel.dot(t1_)));
        amin = std::min(amin, a[i]); amax = std::max(amax, a[i]);
        smin = std::min(smin, s[i]); smax = std::max(smax, s[i]);
    }
    const double da = amax - amin, ds = smax - smin;
    if (da < 4.0 || ds < 4.0) {
        printDebugInfo("点群展开尺寸过小，跳过椭圆拟合");
        return false;
    }

    // 2. 栅格化(尺寸自适应: 约 2x 平均点距, 下限 1mm)
    const double spacing = std::sqrt(da * ds / static_cast<double>(np));
    const double cell = std::max(1.0, 2.2 * spacing);
    const int na = std::max(4, static_cast<int>(std::ceil(da / cell)) + 1);
    const int ns = std::max(4, static_cast<int>(std::ceil(ds / cell)) + 1);
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(na) * ns, 0);
    for (std::size_t i = 0; i < np; ++i) {
        const int ia = std::min(na - 1, std::max(0, static_cast<int>((a[i] - amin) / cell)));
        const int is = std::min(ns - 1, std::max(0, static_cast<int>((s[i] - smin) / cell)));
        mask[static_cast<std::size_t>(ia) * ns + is] = 1;
    }

    // 3. 最大连通域(8连通, BFS), 剔除零散噪点格
    std::vector<int> comp(static_cast<std::size_t>(na) * ns, 0);
    int bestComp = 0, bestSize = 0, compId = 0;
    const int d8[8][2] = { {-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1} };
    for (int ia = 0; ia < na; ++ia) {
        for (int is = 0; is < ns; ++is) {
            const std::size_t idx = static_cast<std::size_t>(ia) * ns + is;
            if (!mask[idx] || comp[idx]) continue;
            ++compId;
            int size = 0;
            std::deque<std::pair<int, int>> dq;
            dq.emplace_back(ia, is);
            comp[idx] = compId;
            while (!dq.empty()) {
                auto [ca, cs] = dq.front();
                dq.pop_front();
                ++size;
                for (const auto& dd : d8) {
                    const int nb_a = ca + dd[0], nb_s = cs + dd[1];
                    if (nb_a < 0 || nb_a >= na || nb_s < 0 || nb_s >= ns) continue;
                    const std::size_t nidx = static_cast<std::size_t>(nb_a) * ns + nb_s;
                    if (mask[nidx] && !comp[nidx]) {
                        comp[nidx] = compId;
                        dq.emplace_back(nb_a, nb_s);
                    }
                }
            }
            if (size > bestSize) {
                bestSize = size;
                bestComp = compId;
            }
        }
    }
    if (bestComp == 0) return false;

    // 4. 边界格: 属于最大连通域且 4 邻域存在空格/边界
    std::vector<Eigen::Vector2d> contour;
    contour.reserve(bestSize);
    for (int ia = 0; ia < na; ++ia) {
        for (int is = 0; is < ns; ++is) {
            const std::size_t idx = static_cast<std::size_t>(ia) * ns + is;
            if (comp[idx] != bestComp) continue;
            const bool edge = ia == 0 || ia == na - 1 || is == 0 || is == ns - 1
                || !mask[idx - ns] || !mask[idx + ns] || !mask[idx - 1] || !mask[idx + 1];
            if (edge) {
                contour.emplace_back(amin + (ia + 0.5) * cell, smin + (is + 0.5) * cell);
            }
        }
    }
    printDebugInfo("栅格边界点数 = " + std::to_string(contour.size())
        + " (栅格 " + std::to_string(na) + "x" + std::to_string(ns)
        + ", cell=" + std::to_string(cell) + "mm)");
    // 4b. 椭圆轮廓:
    //   口径B(局部基准, 判凹坑用): 改用点群凸包. 栅格边界是"阈值掩膜的边界",
    //     相对真实凹坑足迹系统性内缩约 1 个格(实测 φ30mm 坑的栅格只有 21x28mm,
    //     φ11mm 坑只有 8x8mm), 直接拟合会把长短轴报小 15%~50%; 凸包给出的是真实
    //     外边界, 与 Python 独立标定用的"点群足迹"口径一致.
    //   口径A(local=0): 保持原有的"栅格边界格"轮廓, 保证与改造前逐位一致.
    //   栅格在两种模式下都仍用于: 面积占比、贴边判定、格数统计.
    bool hullUsed = false;
    if (use_local_baseline_ && local_ok_) {
        // 口径B: 用点群凸包当轮廓. 门槛只要够拟合 5 个椭圆参数即可,
        // 定成 16 会让 φ11mm 的小坑(凸包 13 点)被直接拒掉, 所以取 8.
        std::vector<Eigen::Vector2d> hull = convexHull2D(a, s);
        if (hull.size() >= 8) {
            contour.swap(hull);
            hullUsed = true;
        }
    }
    // 口径A(栅格边界)沿用原来的 16 点门槛, 保证与改造前逐位一致
    const std::size_t kMinContour = (use_local_baseline_ && local_ok_) ? 8 : 16;
    if (contour.size() < kMinContour) {
        printDebugInfo("边界点过少，跳过椭圆拟合");
        return false;
    }
    printDebugInfo(std::string("椭圆轮廓 = ") + (hullUsed ? "点群凸包" : "栅格边界")
        + " " + std::to_string(contour.size()) + " 点");

    // 5. 最小二乘椭圆拟合
    Eigen::Vector2d ctr;
    double sm1 = 0.0, sm2 = 0.0, ang = 0.0;
    if (!fitEllipseLSQ(contour, ctr, sm1, sm2, ang)) {
        printDebugInfo("椭圆拟合失败(点群轮廓可能非椭圆)");
        return false;
    }
    // 阈值掩膜内缩修正: 判定用 d > T_local, 对抛物面剖面 q<1-D/T 才入选, 故候选点群
    // 只覆盖坑的中部, 由它拟合出的椭圆半轴会系统性偏小(本样本实测 4%~22%).
    // 该偏差与 T_local/坑深 之比相关, 是口径定义固有的量, 这里按最大坑率定一个
    // 常数补偿因子并在报告中同时给出未补偿值, 便于复核与替换.
    //   (T_local=0.35mm 时实测/真值: 28.7/30, 16.4/20, 12.5/16, 9.3/11)
    // 口径A(local=0)路径不做补偿, 保证与改造前逐位一致.
    // 取 1.0 表示"不做补偿": 逐位保留拟合原值, 偏差在报告中如实给出.
    // (本样本上 T_local=0.35 时原值偏差 -4%~-18%, 补偿 1.2 后为 -7%~+15%;
    //  两者都在同一量级, 因此选择不引入经验因子, 保证口径定义干净.)
    const double kFootprintCalib = 1.0;
    ellipse_major_ = kFootprintCalib * 2.0 * sm1;
    ellipse_minor_ = kFootprintCalib * 2.0 * sm2;
    ellipse_major_raw_ = 2.0 * sm1;
    ellipse_minor_raw_ = 2.0 * sm2;
    ellipse_cx_ = ctr.x(); ellipse_cy_ = ctr.y(); ellipse_angle_ = ang;
    ellipse_aspect_ = (sm2 > 1e-9) ? (sm1 / sm2) : 0.0;
    // 椭圆轴向/周向跨度(旋转椭圆的轴对齐外接范围) -> 是否越出补丁
    {
        const double c = std::cos(ang), s2 = std::sin(ang);
        const double half_a = std::sqrt(sm1 * sm1 * c * c + sm2 * sm2 * s2 * s2);
        const double half_s = std::sqrt(sm1 * sm1 * s2 * s2 + sm2 * sm2 * c * c);
        ellipse_out_of_patch_ = (ctr.x() - half_a < patch_a_min_) || (ctr.x() + half_a > patch_a_max_)
            || (ctr.y() - half_s < patch_s_min_) || (ctr.y() + half_s > patch_s_max_);
    }
    printDebugInfo("椭圆拟合: 长轴 " + std::to_string(ellipse_major_)
        + " mm, 短轴 " + std::to_string(ellipse_minor_)
        + " mm, 长宽比 " + std::to_string(ellipse_aspect_));
    // 点群面积占比(栅格占据面积 / 补丁面积) 与 是否贴补丁边界
    pit_area_fraction_ = (bestSize * cell * cell) / std::max(patch_area_, 1e-6);
    {
        const double tol_a = std::max(2.0 * cell, 0.02 * (patch_a_max_ - patch_a_min_));
        const double tol_s = std::max(2.0 * cell, 0.02 * (patch_s_max_ - patch_s_min_));
        pit_touch_boundary_ = (amin - patch_a_min_ < tol_a) || (patch_a_max_ - amax < tol_a)
            || (smin - patch_s_min_ < tol_s) || (patch_s_max_ - smax < tol_s);
    }
    printDebugInfo("点群面积占比 = " + std::to_string(pit_area_fraction_ * 100.0)
        + "%, 贴补丁边界: " + std::string(pit_touch_boundary_ ? "是" : "否")
        + ", 椭圆越界: " + std::string(ellipse_out_of_patch_ ? "是" : "否"));

    // 6. 椭圆回投柱面(闭合折线采样)
    const int kSamples = 240;
    const double kTwoPi = 6.28318530717958647692;
    ellipse_points_.reserve(kSamples);
    const double ca = std::cos(ang), sa = std::sin(ang);
    for (int k = 0; k < kSamples; ++k) {
        const double th = kTwoPi * k / kSamples;
        const double ex = sm1 * std::cos(th);
        const double ey = sm2 * std::sin(th);
        const double ax = ctr.x() + ex * ca - ey * sa;
        const double sy = ctr.y() + ex * sa + ey * ca;
        const double phi = sy / design_radius_;
        Eigen::Vector3f p = axis_point_
            + axis_direction_ * static_cast<float>(ax)
            + t1_ * static_cast<float>(design_radius_ * std::cos(phi))
            + t2_ * static_cast<float>(design_radius_ * std::sin(phi));
        ellipse_points_.push_back(p);
    }
    return true;
}

// ============================================================
// 步骤4: 点群合理性校验(四道闸门)
// 真实凹塘应当是"位于补丁内部、占比不大、形状不细长、椭圆落在补丁内"的局部凹陷;
// 若点群贴补丁边界 / 占了大半面积 / 是细长条 / 椭圆越界, 多半是参考面偏差、
// 边界缺失或形面趋势的残影, 不应判定为凹塘(仍会报告最大距离与最深点).
// ============================================================
bool MeasurePothole::validatePit(std::string& reason)
{
    std::ostringstream ss;
    if (pit_touch_boundary_) {
        reason = "点群贴到补丁边界(疑似参考面偏差/边界缺失区域, 而非内部凹塘)";
        return false;
    }
    if (pit_area_fraction_ > max_area_fraction_) {
        ss << "点群面积占补丁 " << std::fixed << std::setprecision(1)
           << pit_area_fraction_ * 100.0 << "%, 超过上限 " << max_area_fraction_ * 100.0
           << "%(非局部凹陷)";
        reason = ss.str();
        return false;
    }
    if (ellipse_aspect_ > max_aspect_ratio_) {
        ss << "椭圆长宽比 " << std::fixed << std::setprecision(1) << ellipse_aspect_
           << " 超过上限 " << max_aspect_ratio_ << "(细长条状, 非凹坑形状)";
        reason = ss.str();
        return false;
    }
    if (ellipse_out_of_patch_) {
        reason = "拟合椭圆越出补丁范围(结果不可信)";
        return false;
    }
    return true;
}

// ============================================================
// 残差热力图(蓝=凹, 白=0, 红=凸)
// 显示场由 heatmap_field_ 选择: 0=局部凹陷深度 d(口径B, 默认), 1=到理想柱面距离 e(口径A)
// 局部场中 d>0 表示比周围低(凹), 因此配色时取 -d, 与口径A 的"负=凹"保持一致.
// ============================================================
void MeasurePothole::generateHeatMapCloud()
{
    heatmap_cloud_->clear();
    const int n = static_cast<int>(input_cloud_->size());
    if (n <= 0) return;
    heatmap_cloud_->points.resize(n);
    heatmap_cloud_->width = static_cast<std::uint32_t>(n);
    heatmap_cloud_->height = 1;
    heatmap_cloud_->is_dense = false;

    const bool useLocal = (heatmap_field_ == 0) && use_local_baseline_ && local_ok_
        && static_cast<int>(local_dent_.size()) == n;
    heatmap_field_used_ = useLocal ? 0 : 1;
    const double vmax = useLocal ? std::max(2.0 * local_threshold_, 0.5)
                                 : std::max(3.0 * robust_sigma_, 0.5);

#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        const auto& pt = (*input_cloud_)[i];
        auto& out = (*heatmap_cloud_)[i];
        out.x = pt.x; out.y = pt.y; out.z = pt.z;
        const double e = useLocal ? -local_dent_[i] : residual_map_[i];
        if (!std::isfinite(e)) {
            out.r = out.g = out.b = 128;
            continue;
        }
        double t = e / vmax;
        if (t > 1.0) t = 1.0;
        if (t < -1.0) t = -1.0;
        if (t >= 0.0) {  // 凸: 白->红
            out.r = 255;
            out.g = static_cast<std::uint8_t>(255.0 * (1.0 - t));
            out.b = static_cast<std::uint8_t>(255.0 * (1.0 - t));
        }
        else {           // 凹: 白->蓝
            const double u = -t;
            out.r = static_cast<std::uint8_t>(255.0 * (1.0 - u));
            out.g = static_cast<std::uint8_t>(255.0 * (1.0 - u));
            out.b = 255;
        }
    }
}

// ============================================================
// 主流程
// ============================================================
MeasurePothole::PitResult MeasurePothole::evaluate()
{
    PitResult result;

    if (!input_cloud_ || input_cloud_->empty()) {
        result.assessment_message = "错误: 点云为空";
        printDebugInfo(result.assessment_message);
        return result;
    }
    if (!cylinder_) {
        result.assessment_message = "错误: 未设置圆柱拟合结果(请先执行圆柱拟合并保存)";
        printDebugInfo(result.assessment_message);
        return result;
    }

    printDebugInfo("开始凹塘测量, 点数: " + std::to_string(input_cloud_->size())
        + ", 理想半径: " + std::to_string(design_radius_) + " mm");

    try {
        // 步骤1: 残差与最深点
        result.fit_ok = computeResiduals();
        if (!result.fit_ok) {
            result.assessment_message = "错误: 残差计算失败";
            return result;
        }
        const auto& dp = (*input_cloud_)[deepest_index_];
        result.deepest_point = dp;
        result.max_depth = -residual_map_[deepest_index_];
        result.robust_sigma = robust_sigma_;
        result.design_radius = design_radius_;
        result.axis_point = axis_point_;
        result.axis_direction = axis_direction_;

        // 步骤1.5: 形面趋势(系统性偏差)估计与扣除
        estimateFormTrend();

        // 步骤1.8: 局部基准(口径B) —— 缺陷判断依据
        local_ok_ = computeLocalBaseline();

        // 步骤2+3+4: 逐簇 <点群 -> 轮廓 -> 椭圆 -> 回投 -> 四道闸门>
        // (多凹坑: 每个满足 min_cluster_size 的簇各给一份结果, 不再只取最大簇)
        const bool hasCluster = extractPitCluster();
        result.cluster_count = cluster_count_;
        result.pit_points = static_cast<int>(pit_cloud_->size());
        result.mean_depth = hasCluster ? pit_mean_depth_ : 0.0;
        result.pits = pit_items_;
        result.pit_count = static_cast<int>(pit_items_.size());
        result.grid_cell = lb_cell_;
        result.local_window = local_window_;

        // 主坑 = 点数最多者(已排序), 它与原有单坑字段保持一致
        const PitItem* mainPit = pit_items_.empty() ? nullptr : &pit_items_.front();
        const bool hasEllipse = (mainPit != nullptr) && mainPit->ellipse_ok;
        result.major_axis = hasEllipse ? mainPit->major_axis : 0.0;
        result.minor_axis = hasEllipse ? mainPit->minor_axis : 0.0;
        std::string rejectReason = mainPit ? mainPit->reject_reason : std::string();
        // valid: 只要有一个坑通过四道闸门即视为检出有效凹塘(报告里逐个给出结论)
        bool pitValid = false;
        int validCount = 0, rejectedCount = 0;
        for (const auto& it : pit_items_) {
            if (it.valid) { pitValid = true; ++validCount; }
            else ++rejectedCount;
        }
        result.valid = pitValid;
        result.trend_removed = trend_removed_;
        result.trend_p2p = trend_p2p_;
        result.trend_ratio = trend_ratio_;
        result.pit_area_fraction = mainPit ? mainPit->area_fraction : 0.0;
        result.pit_touch_boundary = mainPit ? mainPit->touch_boundary : false;

        // 局部基准(口径B)结果
        result.judged_by_local = (use_local_baseline_ && local_ok_);
        result.local_threshold = result.judged_by_local ? local_threshold_ : 0.0;
        // 兼容字段: 主坑的局部深度(判定区全局最大下凹见 report 的"局部最大下凹深度")
        result.local_max_depth = mainPit ? mainPit->local_max_depth : local_max_depth_;
        result.local_mean_depth = mainPit ? mainPit->local_mean_depth : local_mean_depth_;
        result.baseline_offset_p2p = baseline_offset_p2p_;
        result.boundary_excluded_fraction = boundary_excluded_fraction_;
        result.local_deepest_point = local_deepest_point_;

        // 热力图
        generateHeatMapCloud();

        // 报告(精简版): 结论先行, 逐坑一行; 中间量不再堆在报告里
        std::stringstream ss;
        ss << std::fixed;
        ss << "=== 凹塘测量报告 ===\n";
        ss << "点数 " << input_cloud_->size()
            << " | 设计半径 " << std::setprecision(1) << design_radius_ << " mm"
            << " | 形面偏差峰峰 " << std::setprecision(2) << trend_p2p_ << " mm"
            << " | 表面噪声 σ " << std::setprecision(3) << robust_sigma_ << " mm\n";
        // 逐坑椭圆拟合会重算自己的栅格 cell 并覆盖 lb_cell_; 报告里要用
        // "局部基准那一次的 cell", 所以先取出来
        const double cellForReport = result.grid_cell;
        if (result.judged_by_local) {
            // 判据与本次使用的参数合并成两行(原先重复写了两次)
            ss << "判据 局部基准(相对周围正常表面) | 阈值 " << std::setprecision(2)
                << local_threshold_ << " mm | 窗口 " << local_window_ << " mm | 格边长 "
                << cellForReport << " mm | 边界不判定 " << std::setprecision(1)
                << boundary_excluded_fraction_ * 100.0 << "%\n";
            ss << "聚类容差 " << std::setprecision(2)
                << (cluster_tolerance_ > 0.0 ? cluster_tolerance_ : 3.0)
                << " mm | 最小点数 " << min_cluster_size_ << "\n\n";
        }
        else {
            const double thrA = (distance_threshold_ > 0.0) ? distance_threshold_
                : std::max(3.0 * robust_sigma_work_, 0.8);
            ss << "判据 到理想柱面距离(全局基准) | 距离阈值 " << std::setprecision(2) << thrA
                << " mm" << (distance_threshold_ > 0.0 ? "(手动)" : "(自动)") << "\n\n";
        }
        ss << std::setprecision(3);
        // ---- 多凹坑: 总数 + 逐坑一行(深度/长短轴/点数/位置/结论) ----
        ss << "共检出 " << pit_items_.size() << " 个凹塘";
        if (!pit_items_.empty()) {
            ss << "（通过 " << validCount << "，未通过 " << rejectedCount << "）";
        }
        ss << "\n";
        if (pit_items_.empty()) {
            ss << "  未检出低于阈值的点群\n";
        }
        for (const auto& it : pit_items_) {
            const double depth = it.judged_by_local ? it.local_max_depth : it.global_max_depth;
            ss << "  坑#" << it.index << (it.is_main ? " 主坑" : "    ")
                << " 深 " << depth << " mm | 椭圆 ";
            if (it.ellipse_ok) ss << it.major_axis << " × " << it.minor_axis << " mm";
            else               ss << "拟合失败";
            ss << " | " << it.pit_points << " 点 | 位置 ";
            if (it.ellipse_ok) ss << "(a,s)=(" << it.ellipse_center_a << ", " << it.ellipse_center_s << ")";
            else               ss << "(a,s)=(" << it.centroid_a << ", " << it.centroid_s << ")";
            ss << " mm | " << (it.valid ? "通过" : ("不判定(" + it.reject_reason + ")")) << "\n";
        }
        // ---- 需求原文要求的全局数值 ----
        ss << "\n到理想柱面最大距离 " << result.max_depth << " mm @ ("
            << dp.x << ", " << dp.y << ", " << dp.z << ")\n";
        // ---- 判定 ----
        ss << "判定 ";
        if (pitValid) {
            ss << "检出有效凹塘（" << validCount << "/" << pit_items_.size() << " 个通过校验）\n";
        }
        else if (pit_items_.empty()) {
            ss << "未检出凹塘\n";
            if (result.judged_by_local && local_max_depth_ > 0.0
                && local_max_depth_ < 1.5 * local_threshold_) {
                ss << "  提示 局部最大下凹 " << local_max_depth_ << " mm 与阈值同量级; "
                    << "若该处确有大尺寸凹坑, 可能是窗口 W=" << local_window_
                    << " mm 偏小(需 W ≥ 2~3 倍坑尺寸)。\n";
            }
        }
        else if (!hasEllipse) {
            ss << "候选点群无法拟合成椭圆, 不判定为凹塘\n";
        }
        else {
            ss << "不判定为凹塘 —— " << rejectReason << "\n";
            ss << "  (到理想柱面的最大距离与位置照常给出)\n";
            ss << "  提示 若确认是真实缺陷, 可放宽相应上限或改用手动阈值。\n";
        }
        result.assessment_message = ss.str();
    }
    catch (const std::exception& e) {
        result.assessment_message = "凹塘测量过程出错: " + std::string(e.what());
    }

    printDebugInfo(result.assessment_message);
    return result;
}

// ============================================================
// 工具函数
// ============================================================
bool MeasurePothole::isPointValid(const pcl::PointXYZ& point) const
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void MeasurePothole::printDebugInfo(const std::string& message) const
{
    if (verbose_) {
        std::cout << "[MeasurePothole] " << message << std::endl;
    }
}
