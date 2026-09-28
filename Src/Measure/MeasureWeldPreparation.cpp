#include "Measure/MeasureWeldPreparation.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>

#include "Measure/MeasureCylindricity.h"

// ============================================================
// MeasureWeldPreparation 实现
// 依据: docs/当前需新增功能/焊前装配阶差与间隙-独立执行方案.md
// 计算类: 不依赖 Qt / VTK / 查看器; 不使用 std::cout; 进度经 ProgressCallback 上报。
// ============================================================

namespace {

constexpr double kPi = 3.14159265358979323846;

inline bool finiteD(double v) { return std::isfinite(v); }
inline bool finiteV(const Eigen::Vector3d& v)
{
    return finiteD(v.x()) && finiteD(v.y()) && finiteD(v.z());
}

std::string fmt(double v, int prec = 3)
{
    if (!std::isfinite(v)) return "N/A";
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(prec);
    oss << v;
    return oss.str();
}

std::string fmt3(const Eigen::Vector3d& p, int prec = 3)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(prec);
    oss << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")";
    return oss.str();
}

// 与 u 垂直的确定性正交基(与 CylinderSurfaceFrame 内部同规则)
void perpBasis(const Eigen::Vector3d& u, Eigen::Vector3d& v, Eigen::Vector3d& w)
{
    const double ax = std::fabs(u.x()), ay = std::fabs(u.y()), az = std::fabs(u.z());
    Eigen::Vector3d helper;
    if (ax <= ay && ax <= az)      helper = Eigen::Vector3d::UnitX();
    else if (ay <= az)             helper = Eigen::Vector3d::UnitY();
    else                           helper = Eigen::Vector3d::UnitZ();
    v = helper - u * helper.dot(u);
    const double n = v.norm();
    v = (n > 1e-12) ? Eigen::Vector3d(v / n) : Eigen::Vector3d::UnitX();
    w = u.cross(v);
    const double nw = w.norm();
    if (nw > 1e-12) { w /= nw; v = w.cross(u).normalized(); }
}

// 二维稳健直线拟合: 主成分定方向 + 残差截尾。
// 这是方案 §4.4 要求的"局部稳健直线细化", 不是用一条全局直线替代整条边缘。
struct Line2D {
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    Eigen::Vector2d d = Eigen::Vector2d::UnitX();
    double sigma = 0.0;   // 参与拟合点的残差 rms(mm)
    bool   ok = false;
};

Line2D fitLine2DRobust(const std::vector<Eigen::Vector2d>& pts)
{
    Line2D L;
    if (pts.size() < 2) return L;
    std::vector<int> idx(pts.size());
    std::iota(idx.begin(), idx.end(), 0);

    for (int iter = 0; iter < 3; ++iter) {
        if (idx.size() < 2) break;
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (int i : idx) mean += pts[i];
        mean /= static_cast<double>(idx.size());

        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (int i : idx) {
            const Eigen::Vector2d d = pts[i] - mean;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(idx.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        if (es.info() != Eigen::Success) return L;
        L.p = mean;
        L.d = Eigen::Vector2d(es.eigenvectors().col(1)).normalized();  // 最大特征值 = 主方向
        L.ok = true;

        std::vector<double> res(idx.size());
        double s2 = 0.0;
        for (size_t k = 0; k < idx.size(); ++k) {
            const Eigen::Vector2d dv = pts[idx[k]] - L.p;
            res[k] = std::fabs(dv.x() * L.d.y() - dv.y() * L.d.x());
            s2 += res[k] * res[k];
        }
        L.sigma = std::sqrt(s2 / static_cast<double>(idx.size()));
        if (iter == 2) break;

        std::vector<double> sorted = res;
        std::sort(sorted.begin(), sorted.end());
        const double med = sorted[sorted.size() / 2];
        const double thr = std::max(2.5 * med, 2.5 * L.sigma);
        std::vector<int> kept;
        for (size_t k = 0; k < idx.size(); ++k) {
            if (res[k] <= thr) kept.push_back(idx[k]);
        }
        if (kept.size() < std::max<size_t>(3, idx.size() / 2)) break;
        idx.swap(kept);
    }
    return L;
}

// o + t·q 与线段 [A,B] 求交; ratio = |q × d| / |d| = |sin(夹角)|
bool raySegment(const Eigen::Vector2d& o, const Eigen::Vector2d& q,
                const Eigen::Vector2d& A, const Eigen::Vector2d& B,
                double& t, double& ratio)
{
    const Eigen::Vector2d d = B - A;
    const double dl = d.norm();
    if (dl < 1e-12) return false;
    const double cr = q.x() * d.y() - q.y() * d.x();      // q × d
    ratio = std::fabs(cr) / dl;
    if (ratio < 1e-12) {
        // 严格平行: t 无定义。必须作为"平行退化"上报, 不能悄悄算成"无交点",
        // 否则纵向直缝按轴向测间隙时报告会变成"无交点", 无法解释真实原因。
        t = 0.0;
        return true;
    }
    const Eigen::Vector2d w = A - o;
    t = (w.x() * d.y() - w.y() * d.x()) / cr;
    const double u = -(q.x() * w.y() - q.y() * w.x()) / cr;
    return (u >= -1e-9 && u <= 1.0 + 1e-9);
}

// 两条空间直线的夹角(度)与公垂线距离(mm)
double lineLineRelation(const Eigen::Vector3d& p1, const Eigen::Vector3d& d1,
                        const Eigen::Vector3d& p2, const Eigen::Vector3d& d2,
                        double& angle_deg)
{
    angle_deg = std::acos(std::min(1.0, std::max(-1.0, std::fabs(d1.dot(d2))))) * 180.0 / kPi;
    const Eigen::Vector3d n = d1.cross(d2);
    const Eigen::Vector3d w = p2 - p1;
    if (n.norm() < 1e-12) {
        return (w - w.dot(d1) * d1).norm();
    }
    return std::fabs(w.dot(n.normalized()));
}

} // namespace

// ============================================================
// 构造 / 输入
// ============================================================
MeasureWeldPreparation::MeasureWeldPreparation() = default;
MeasureWeldPreparation::~MeasureWeldPreparation() = default;

void MeasureWeldPreparation::setNearCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          const std::string& name)
{
    near_cloud_ = std::move(cloud);
    near_name_ = name;
    near_indices_.clear();
    single_mode_ = false;
}

void MeasureWeldPreparation::setFarCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         const std::string& name)
{
    far_cloud_ = std::move(cloud);
    far_name_ = name;
    far_indices_.clear();
    single_mode_ = false;
}

void MeasureWeldPreparation::setSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                            const std::string& name)
{
    single_cloud_ = std::move(cloud);
    single_name_ = name;
    single_mode_ = true;
    near_cloud_ = single_cloud_;
    far_cloud_ = single_cloud_;
    near_indices_.clear();
    far_indices_.clear();
}

void MeasureWeldPreparation::setNearIndices(std::vector<int> indices)
{
    near_indices_ = std::move(indices);
    if (single_mode_) near_cloud_ = single_cloud_;
}

void MeasureWeldPreparation::setFarIndices(std::vector<int> indices)
{
    far_indices_ = std::move(indices);
    if (single_mode_) far_cloud_ = single_cloud_;
}

void MeasureWeldPreparation::setSeamRoi(double a0, double a1, double s0, double s1)
{
    if (a1 < a0) std::swap(a0, a1);
    if (s1 < s0) std::swap(s0, s1);
    roi_a0_ = a0; roi_a1_ = a1; roi_s0_ = s0; roi_s1_ = s1;
    roi_set_ = (a1 > a0) && (s1 > s0);
}

void MeasureWeldPreparation::setRoleOverride(Role near, Role far, const std::string& reason)
{
    if (near == far) return;
    role_override_ = true;
    role_near_ = near;
    role_far_ = far;
    role_override_reason_ = reason;
}

void MeasureWeldPreparation::clearRoleOverride()
{
    role_override_ = false;
    role_override_reason_.clear();
}

void MeasureWeldPreparation::setAxisPrior(const Eigen::Vector3d& point,
                                          const Eigen::Vector3d& direction)
{
    if (!finiteV(point) || !finiteV(direction) || direction.norm() < 1e-9) return;
    axis_prior_set_ = true;
    prior_point_ = point;
    prior_dir_ = direction.normalized();
}

void MeasureWeldPreparation::setFixedReferenceAxis(const Eigen::Vector3d& point,
                                                   const Eigen::Vector3d& direction)
{
    if (!finiteV(point) || !finiteV(direction) || direction.norm() < 1e-9) return;
    axis_fixed_ = true;
    fixed_point_ = point;
    fixed_dir_ = direction.normalized();
}

bool MeasureWeldPreparation::reportProgress(int current, int total, const std::string& stage)
{
    if (cancelled_) return false;
    if (!progress_cb_) return true;
    if (!progress_cb_(current, total, stage)) {
        cancelled_ = true;
        return false;
    }
    return true;
}

// ============================================================
// 输入与参数校验(§4.2 "参数有限性、R>0、轴向长度、取消状态及结果有效性要独立检查")
// ============================================================
bool MeasureWeldPreparation::checkInputs(std::string& reason) const
{
    if (!finiteD(params_.design_radius) || !(params_.design_radius > 0.0)) {
        reason = "设计半径 R 必须为正的有限值(由用户按图纸提供, 不由本测量自动推断)";
        return false;
    }
    if (params_.edge_directions < 8 || params_.edge_directions > 720) {
        reason = "边缘邻域方向数超出允许范围(8~720)";
        return false;
    }
    if (!(params_.boundary_gap_deg > 0.0 && params_.boundary_gap_deg < 360.0)) {
        reason = "边界判据的角度空缺阈值必须在 (0, 360) 度内";
        return false;
    }
    if (!(params_.radius_min_factor >= 0.0) ||
        !(params_.radius_max_factor > params_.radius_min_factor)) {
        reason = "边缘邻域半径系数非法(需 0 ≤ r_min < r_max)";
        return false;
    }
    if (!(params_.min_edge_support >= 0.0 && params_.min_edge_support <= 1.0)) {
        reason = "边缘支持比例下限必须在 [0,1] 内";
        return false;
    }
    if (single_mode_) {
        if (!single_cloud_ || single_cloud_->empty()) {
            reason = "单点云模式: 输入点云为空";
            return false;
        }
        if (near_indices_.empty() || far_indices_.empty()) {
            reason = "单点云模式: 必须先人工圈定近件与远件两个试样的点集";
            return false;
        }
    }
    else {
        if (!near_cloud_ || near_cloud_->empty()) { reason = "近件点云为空"; return false; }
        if (!far_cloud_ || far_cloud_->empty()) { reason = "远件点云为空"; return false; }
    }
    return true;
}

// ============================================================
// §3 近远判定: 先确定两个试样, 再比较各自 Z 均值; Z 均值只用于命名
// ============================================================
bool MeasureWeldPreparation::resolveRoles(Result& r, std::string& reason)
{
    auto zMean = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& c,
                    const std::vector<int>& idx, int& count) -> double {
        double sum = 0.0;
        count = 0;
        if (!c) return 0.0;
        if (idx.empty()) {
            for (const auto& p : c->points) {
                if (!pcl::isFinite(p)) continue;
                sum += p.z; ++count;
            }
        }
        else {
            for (int i : idx) {
                if (i < 0 || i >= static_cast<int>(c->size())) continue;
                const auto& p = c->points[i];
                if (!pcl::isFinite(p)) continue;
                sum += p.z; ++count;
            }
        }
        return count > 0 ? sum / count : 0.0;
    };

    int nA = 0, nB = 0;
    const double zA = zMean(near_cloud_, near_indices_, nA);
    const double zB = zMean(far_cloud_, far_indices_, nB);
    if (nA == 0 || nB == 0) {
        reason = "其中一个试样的有效点数为 0, 无法比较 Z 均值";
        return false;
    }

    bool swapNeeded = false;
    if (role_override_) {
        // 人工指定: role_near_ 描述"第一个输入槽位"应当扮演的角色
        swapNeeded = (role_near_ == Role::Far);
        r.role_overridden = true;
        r.role_note = "已人工指定近远角色";
        if (!role_override_reason_.empty()) r.role_note += ": " + role_override_reason_;
    }
    else if (!z_convention_known_) {
        reason = "扫描坐标约定未知, 无法判定近远件; 需先指定近远角色";
        return false;
    }
    else {
        const bool aIsNear = z_increase_away_ ? (zA < zB) : (zA > zB);
        swapNeeded = !aIsNear;
        const double dz = std::fabs(zA - zB);
        r.role_note = std::string("按 Z 均值自动命名(约定: Z 增大代表")
            + (z_increase_away_ ? "远离" : "靠近") + "扫描仪); ΔZ=" + fmt(dz, 3) + " mm";
        if (dz < 1e-3) {
            reason = "两试样的 Z 均值几乎相同, 无法判定近远件; 需先指定近远角色";
            return false;
        }
    }

    if (swapNeeded) {
        // 交换的是"哪个点是近件", 不是坐标变换: 原始坐标与源索引保持不变
        std::swap(near_cloud_, far_cloud_);
        std::swap(near_indices_, far_indices_);
        std::swap(near_name_, far_name_);
    }

    r.near_z_mean = swapNeeded ? zB : zA;
    r.far_z_mean = swapNeeded ? zA : zB;
    r.near_points = swapNeeded ? nB : nA;
    r.far_points = swapNeeded ? nA : nB;
    r.near_role = Role::Near;
    r.near_cloud_name = near_name_.empty() ? std::string("试样 A") : near_name_;
    r.far_cloud_name = far_name_.empty() ? std::string("试样 B") : far_name_;
    r.far_role = Role::Far;
    return true;
}

// ============================================================
// 私有工具: 点距估计(用于栅格尺度自动取值)
// ============================================================
double MeasureWeldPreparation::estimatePointPitch(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                  const std::vector<int>& indices) const
{
    std::vector<Eigen::Vector3d> sample;
    const int total = static_cast<int>(indices.empty() ? cloud.size() : indices.size());
    if (total <= 1) return 1.0;
    const int want = std::min(1200, total);
    const int stride = std::max(1, total / want);
    sample.reserve(want);
    for (int k = 0; k < total && static_cast<int>(sample.size()) < want; k += stride) {
        const int i = indices.empty() ? k : indices[k];
        if (i < 0 || i >= static_cast<int>(cloud.size())) continue;
        const auto& p = cloud.points[i];
        if (!pcl::isFinite(p)) continue;
        sample.emplace_back(p.x, p.y, p.z);
    }
    if (sample.size() < 3) return 1.0;

    std::vector<double> nn;
    nn.reserve(sample.size());
    for (size_t i = 0; i < sample.size(); ++i) {
        double best = std::numeric_limits<double>::max();
        for (size_t j = 0; j < sample.size(); ++j) {
            if (i == j) continue;
            const double d = (sample[i] - sample[j]).squaredNorm();
            if (d < best) best = d;
        }
        if (best < std::numeric_limits<double>::max()) nn.push_back(std::sqrt(best));
    }
    if (nn.empty()) return 1.0;
    std::sort(nn.begin(), nn.end());
    double med = nn[nn.size() / 2];
    if (!(med > 1e-6)) med = 1.0;
    return med;
}

// ============================================================
// 私有工具: 固定半径下的轴线残差与局部精化
// (evaluateCylindricity 的 400 方向全局搜索代价高, 不能用于逐子区稳定性检查)
// ============================================================
double MeasureWeldPreparation::axisRms(const std::vector<Eigen::Vector3d>& pts,
                                       const Eigen::Vector3d& c,
                                       const Eigen::Vector3d& u) const
{
    if (pts.empty()) return std::numeric_limits<double>::max();
    const double R = params_.design_radius;
    double s2 = 0.0;
    for (const auto& p : pts) {
        const Eigen::Vector3d d = p - c;
        const double along = d.dot(u);
        const double rho = (d - along * u).norm();
        const double e = rho - R;
        s2 += e * e;
    }
    return std::sqrt(s2 / static_cast<double>(pts.size()));
}

double MeasureWeldPreparation::refineAxis(const std::vector<Eigen::Vector3d>& pts,
                                          Eigen::Vector3d& c, Eigen::Vector3d& u,
                                          double cStep, double uStepDeg, int rounds) const
{
    if (pts.size() < 10) return axisRms(pts, c, u);
    double best = axisRms(pts, c, u);
    for (int round = 0; round < rounds; ++round) {
        for (int inner = 0; inner < 3; ++inner) {
            Eigen::Vector3d v, w;
            perpBasis(u, v, w);
            bool improved = false;
            for (int k = 0; k < 8; ++k) {
                Eigen::Vector3d c2 = c, u2 = u;
                if (k < 4) {
                    const Eigen::Vector3d dir = (k == 0) ? v : (k == 1) ? -v : (k == 2) ? w : -w;
                    c2 = c + cStep * dir;
                }
                else {
                    const int kk = k - 4;
                    const Eigen::Vector3d ax = (kk == 0) ? v : (kk == 1) ? -v : (kk == 2) ? w : -w;
                    u2 = (u + (uStepDeg * kPi / 180.0) * ax).normalized();
                }
                const double e = axisRms(pts, c2, u2);
                if (e < best - 1e-12) {
                    best = e; c = c2; u = u2;
                    improved = true;
                }
            }
            if (!improved) break;
        }
        cStep *= 0.5;
        uStepDeg *= 0.5;
    }
    return best;
}

// ============================================================
// §4.2 建立远件圆柱基准
// ============================================================
bool MeasureWeldPreparation::buildReferenceCylinder(Result& r, std::string& reason)
{
    // ---- 1. 收集远件三维点(保留原始索引) ----
    std::vector<Eigen::Vector3d> pts;
    const auto& fc = *far_cloud_;
    const int total = single_mode_ ? static_cast<int>(far_indices_.size())
                                   : static_cast<int>(fc.size());
    pts.reserve(total);
    for (int k = 0; k < total; ++k) {
        const int i = single_mode_ ? far_indices_[k] : k;
        if (i < 0 || i >= static_cast<int>(fc.size())) continue;
        const auto& p = fc.points[i];
        if (!pcl::isFinite(p)) continue;
        pts.emplace_back(p.x, p.y, p.z);
    }
    if (pts.size() < 200) {
        reason = "远件有效点数不足(" + std::to_string(pts.size()) +
                 " 点), 无法建立参考圆柱基准";
        return false;
    }

    // ---- 1b. 外部权威轴线: 直接采用, 不做远件拟合(用于 A/B 层验证, 或现场有可靠设计轴时) ----
    if (axis_fixed_) {
        Eigen::Vector3d cf = fixed_point_, uf = fixed_dir_;
        if (!finiteV(cf) || !finiteV(uf) || uf.norm() < 1e-9) {
            reason = "外部给定轴线参数非法";
            return false;
        }
        uf.normalize();
        CylinderSurfaceFrame fx;
        fx.init(cf, uf, params_.design_radius, 0.0);
        std::vector<double> e(pts.size());
        double sum = 0.0;
        for (size_t k = 0; k < pts.size(); ++k) {
            double a = 0, sd = 0, ev = 0;
            fx.project(pts[k], a, sd, ev);
            e[k] = ev;
            sum += ev;
        }
        std::sort(e.begin(), e.end());
        fit_mean_ = e.empty() ? 0.0 : sum / static_cast<double>(e.size());
        fit_p2p_ = e.empty() ? 0.0 : (e.back() - e.front());
        double s2 = 0.0;
        for (double v : e) s2 += v * v;
        fit_rms_ = e.empty() ? 0.0 : std::sqrt(s2 / static_cast<double>(e.size()));
        axis_stab_deg_ = 0.0;
        axis_stab_mm_ = 0.0;
        r.axis_point = cf;
        r.axis_direction = uf;
        r.fit_rms = fit_rms_;
        r.fit_mean = fit_mean_;
        r.fit_p2p = fit_p2p_;
        r.axis_stability_deg = 0.0;
        r.axis_stability_mm = 0.0;
        r.fit_points = static_cast<int>(pts.size());
        r.fit_ok = true;
        r.baseline_reliable = true;
        {
            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss.precision(4);
            oss << "参考圆柱基准: 使用外部给定轴线(未做远件拟合, 未做子区稳定性检查); "
                << "远件残余 RMS " << fit_rms_ << " mm, 均值 " << fit_mean_
                << " mm, 峰峰值 " << fit_p2p_ << " mm";
            r.baseline_note = oss.str();
        }
        return true;
    }

    // ---- 2. 复用 MeasureCylindricity 的固定半径优化接口 ----
    // 只调 evaluateCylindricity; 明确不调 evaluateCylindricityWithWeld(§4.2)。
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitCloud(new pcl::PointCloud<pcl::PointXYZ>);
    fitCloud->points.reserve(pts.size());
    for (const auto& p : pts) {
        fitCloud->points.emplace_back(static_cast<float>(p.x()),
                                      static_cast<float>(p.y()),
                                      static_cast<float>(p.z()));
    }
    fitCloud->width = static_cast<std::uint32_t>(fitCloud->points.size());
    fitCloud->height = 1;
    fitCloud->is_dense = true;

    if (!reportProgress(1, kStageTotal, "焊前测量: 拟合参考圆柱...")) {
        reason = "已取消";
        return false;
    }

    MeasureCylindricity mc;
    mc.setInputCloud(fitCloud);
    mc.setDesignRadius(params_.design_radius);
    mc.setVerbose(params_.verbose);
    mc.setProgressCallback([this](int c, int t, const std::string& s) {
        return reportProgress(c, t, s);
    });
    if (axis_prior_set_) {
        mc.setInitialLine(prior_point_.cast<float>(), prior_dir_.cast<float>());
        r.axis_prior_used = true;
    }
    mc.evaluateCylindricity();
    if (cancelled_) { reason = "已取消"; return false; }

    // 取回轴线前的两道独立闸门(见 docs/当前需新增功能/焊前装配阶差与间隙-工作交接与查验清单.md BUG-1):
    //   (1) "这次评估真的算过"的标志。evaluateCylindricity() 在输入非法、用户取消或内部异常时
    //       会**提前返回**, 既不改写 last_optimized_line_, 也不调 computeAssessmentMetrics()。
    //       而 computeAssessmentMetrics() 是唯一把 design_radius 写进结果的地方, 因此
    //       design_radius > 0 是可靠的"算过"证据。没有它, 下面会把默认轴线(过原点、方向 +Z)
    //       当成一次成功的拟合结果 —— 那根轴在数据上看起来完全"合法"。
    //   (2) 与实现无关的独立兜底: 用 double 自己算一遍径向 RMS。默认轴线在真实数据上会给出
    //       与 R 同量级的残差, 而一次正常拟合远小于 2%R。
    auto tryGetAxis = [&](Eigen::Vector3d& c, Eigen::Vector3d& u, std::string& why) -> bool {
        if (!(mc.getLastAssessmentResult().getDesignRadius() > 0.0)) {
            why = "远件固定半径拟合未完成(输入非法/被取消/内部异常), 没有可信的轴线输出";
            return false;
        }
        const auto lp = mc.getOptimizedAxisParams();
        c = lp.point.cast<double>();
        u = lp.direction.cast<double>();
        if (!finiteV(c) || !finiteV(u) || u.norm() < 1e-9) {
            why = "轴线参数非法(非有限值或方向退化为零)";
            return false;
        }
        u.normalize();
        const double rms = axisRms(pts, c, u);
        const double rmsLimit = std::max(0.5, 0.02 * params_.design_radius);
        if (!(rms <= rmsLimit)) {
            why = "轴线残差 " + fmt(rms, 3) + " mm 超过容许的 " + fmt(rmsLimit, 3)
                + " mm(2%×R), 该轴线不能作为参考基准";
            return false;
        }
        return true;
    };
    Eigen::Vector3d c, u;
    std::string axisWhy;
    if (!tryGetAxis(c, u, axisWhy)) {
        reason = "远件参考圆柱基准不可用(" + axisWhy + ")";
        return false;
    }

    // ---- 3. 排除接缝附近的远件点(§4.1) ----
    CylinderSurfaceFrame tmpFrame;
    tmpFrame.init(c, u, params_.design_radius, 0.0);

    // 接缝附近的排除带宽: 默认 20 mm(与 1940 mm 级筒体、mm 级形面偏差的尺度相称),
    // 不依赖此时尚未可靠估计的点距; 需要时由用户在参数里显式给出。
    const double seamExclude = params_.seam_exclude > 0 ? params_.seam_exclude : 20.0;
    const double gridCell = params_.grid_cell > 0
        ? params_.grid_cell : std::max(2.0 * point_pitch_, 0.5);

    std::vector<Eigen::Vector2d> nearAS;
    {
        const auto& nc = *near_cloud_;
        const int nTotal = single_mode_ ? static_cast<int>(near_indices_.size())
                                        : static_cast<int>(nc.size());
        nearAS.reserve(nTotal);
        for (int k = 0; k < nTotal; ++k) {
            const int i = single_mode_ ? near_indices_[k] : k;
            if (i < 0 || i >= static_cast<int>(nc.size())) continue;
            const auto& p = nc.points[i];
            if (!pcl::isFinite(p)) continue;
            double a = 0, s = 0, e = 0;
            if (!tmpFrame.project(Eigen::Vector3d(p.x, p.y, p.z), a, s, e)) continue;
            nearAS.emplace_back(a, s);
        }
    }

    std::vector<Eigen::Vector3d> keptPts;
    keptPts.reserve(pts.size());
    std::string extraNote;
    {
        // nearAS 空间栅格(加速最近点查询)
        const double extra = std::max(2.0 * gridCell, 1e-3);
        std::unordered_map<long long, std::vector<int>> bucket;
        auto key = [](int ia, int is) {
            return (static_cast<long long>(ia) << 32) ^ static_cast<long long>(is & 0xffffffff);
        };
        for (size_t k = 0; k < nearAS.size(); ++k) {
            const int ia = static_cast<int>(std::floor(nearAS[k].x() / extra));
            const int is = static_cast<int>(std::floor(nearAS[k].y() / extra));
            bucket[key(ia, is)].push_back(static_cast<int>(k));
        }
        auto nearDist = [&](double a, double s) -> double {
            const int ia = static_cast<int>(std::floor(a / extra));
            const int is = static_cast<int>(std::floor(s / extra));
            double best = std::numeric_limits<double>::max();
            const int reach = static_cast<int>(std::ceil(seamExclude / extra)) + 1;
            for (int da = -reach; da <= reach; ++da) {
                for (int ds = -reach; ds <= reach; ++ds) {
                    auto it = bucket.find(key(ia + da, is + ds));
                    if (it == bucket.end()) continue;
                    for (int k : it->second) {
                        const double dda = a - nearAS[k].x();
                        const double dds = tmpFrame.arcDifference(s, nearAS[k].y());
                        const double d2 = dda * dda + dds * dds;
                        if (d2 < best) best = d2;
                    }
                }
            }
            return best < std::numeric_limits<double>::max() ? std::sqrt(best)
                                                             : std::numeric_limits<double>::max();
        };

        for (const auto& P : pts) {
            double a = 0, s = 0, e = 0;
            if (!tmpFrame.project(P, a, s, e)) continue;
            if (nearAS.empty() || nearDist(a, s) > seamExclude) keptPts.push_back(P);
        }
    }
    if (keptPts.size() < 200) {
        // 排除过度(例如两件几乎贴合)时退回全量, 但明确记录(不静默降级)
        keptPts = pts;
        extraNote = "接缝附近排除后剩余点过少, 已退回使用远件全量点拟合(基准可靠性下降)";
    }

    // ---- 4. 稳健截尾 + 局部精化 ----
    double rms = refineAxis(keptPts, c, u, 5.0, 0.5, 6);
    int iter = 0;
    for (; iter < 3; ++iter) {
        std::vector<double> e(keptPts.size());
        for (size_t k = 0; k < keptPts.size(); ++k) {
            const Eigen::Vector3d d = keptPts[k] - c;
            e[k] = (d - d.dot(u) * u).norm() - params_.design_radius;
        }
        std::vector<double> sorted = e;
        std::sort(sorted.begin(), sorted.end());
        const double med = sorted[sorted.size() / 2];
        std::vector<double> dev(sorted.size());
        for (size_t k = 0; k < sorted.size(); ++k) dev[k] = std::fabs(sorted[k] - med);
        std::sort(dev.begin(), dev.end());
        double sigma = 1.4826 * dev[dev.size() / 2];
        if (!(sigma > 1e-9)) sigma = std::max(1e-6, rms);
        const double thr = std::max(3.0 * sigma, 0.05);

        std::vector<Eigen::Vector3d> kept2;
        kept2.reserve(keptPts.size());
        for (size_t k = 0; k < keptPts.size(); ++k) {
            if (std::fabs(e[k] - med) <= thr) kept2.push_back(keptPts[k]);
        }
        if (kept2.size() < std::max<size_t>(200, keptPts.size() * 7 / 10)) break;
        keptPts.swap(kept2);
        rms = refineAxis(keptPts, c, u, 2.5, 0.25, 4);
    }
    r.fit_iterations = iter;

    // ---- 5. 最终拟合统计 ----
    {
        std::vector<double> e(keptPts.size());
        double sum = 0.0;
        for (size_t k = 0; k < keptPts.size(); ++k) {
            const Eigen::Vector3d d = keptPts[k] - c;
            e[k] = (d - d.dot(u) * u).norm() - params_.design_radius;
            sum += e[k];
        }
        std::sort(e.begin(), e.end());
        fit_mean_ = e.empty() ? 0.0 : sum / static_cast<double>(e.size());
        fit_p2p_ = e.empty() ? 0.0 : (e.back() - e.front());
        double s2 = 0.0;
        for (double v : e) s2 += v * v;
        fit_rms_ = e.empty() ? 0.0 : std::sqrt(s2 / static_cast<double>(e.size()));
    }

    // ---- 6. 轴线稳定性(§4.2 "不能以 RMS 小就宣布轴线可靠") ----
    {
        CylinderSurfaceFrame f;
        f.init(c, u, params_.design_radius, 0.0);
        std::vector<double> av(keptPts.size()), sv(keptPts.size()), ev(keptPts.size());
        for (size_t k = 0; k < keptPts.size(); ++k) f.project(keptPts[k], av[k], sv[k], ev[k]);

        double aLo = std::numeric_limits<double>::max(), aHi = -std::numeric_limits<double>::max();
        double sLo = std::numeric_limits<double>::max(), sHi = -std::numeric_limits<double>::max();
        for (size_t k = 0; k < keptPts.size(); ++k) {
            aLo = std::min(aLo, av[k]); aHi = std::max(aHi, av[k]);
            sLo = std::min(sLo, sv[k]); sHi = std::max(sHi, sv[k]);
        }
        r.fit_coverage_a = (aHi > aLo) ? (aHi - aLo) : 0.0;

        double maxDeg = 0.0, maxMm = 0.0;
        int subOk = 0;
        for (int band = 0; band < 3; ++band) {
            const double lo = aLo + (aHi - aLo) * band / 3.0;
            const double hi = aLo + (aHi - aLo) * (band + 1) / 3.0;
            for (int half = 0; half < 2; ++half) {
                std::vector<Eigen::Vector3d> sub;
                for (size_t k = 0; k < keptPts.size(); ++k) {
                    if (av[k] < lo || av[k] > hi) continue;
                    const bool upper = sv[k] >= (sLo + sHi) * 0.5;
                    if ((half == 1) != upper) continue;
                    sub.push_back(keptPts[k]);
                }
                if (sub.size() < 200) continue;
                Eigen::Vector3d cs = c, us = u;
                refineAxis(sub, cs, us, 3.0, 0.3, 4);
                double ang = 0.0;
                const double dd = lineLineRelation(c, u, cs, us, ang);
                maxDeg = std::max(maxDeg, ang);
                maxMm = std::max(maxMm, dd);
                ++subOk;
            }
        }
        axis_stab_deg_ = maxDeg;
        axis_stab_mm_ = maxMm;
        if (subOk == 0) {
            extraNote += (extraNote.empty() ? "" : "; ");
            extraNote += "远件覆盖不足, 无法完成子区稳定性检查(基准稳定性未获独立确认)";
        }
    }
    baseline_note_ = extraNote;

    r.axis_point = c;
    r.axis_direction = u;
    r.fit_rms = fit_rms_;
    r.fit_mean = fit_mean_;
    r.fit_p2p = fit_p2p_;
    r.axis_stability_deg = axis_stab_deg_;
    r.axis_stability_mm = axis_stab_mm_;
    r.fit_points = static_cast<int>(keptPts.size());
    r.fit_ok = true;

    const bool stable = (axis_stab_deg_ <= params_.max_axis_stability_deg) &&
                        (axis_stab_mm_ <= params_.max_axis_stability_mm);
    r.baseline_reliable = stable;
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(4);
        oss << "远件固定半径(R=" << params_.design_radius << " mm)拟合: 点数 " << keptPts.size()
            << ", 稳健剔除 " << r.fit_iterations << " 轮, RMS " << fit_rms_
            << " mm, 均值 " << fit_mean_ << " mm, 峰峰值 " << fit_p2p_
            << " mm, 轴向覆盖 " << r.fit_coverage_a << " mm; 子区稳定性: 轴向最大夹角 "
            << axis_stab_deg_ << "°(阈值 " << params_.max_axis_stability_deg
            << "°), 轴线最大偏移 " << axis_stab_mm_ << " mm(阈值 "
            << params_.max_axis_stability_mm << " mm)";
        if (!extraNote.empty()) oss << "; " << extraNote;
        r.baseline_note = oss.str();
    }

    if (!stable) {
        reason = "参考圆柱基准不稳定(" + fmt(axis_stab_deg_, 4) + "° / "
                 + fmt(axis_stab_mm_, 4) + " mm)";
        return false;
    }
    return true;
}

// ============================================================
// §2.1 建立共同参考圆柱坐标系并展开两件
// ============================================================
bool MeasureWeldPreparation::buildFrameAndProject(Result& r, std::string& reason)
{
    if (!reportProgress(2, kStageTotal, "焊前测量: 展开两件点云...")) {
        reason = "已取消";
        return false;
    }

    const Eigen::Vector3d c = r.axis_point;
    const Eigen::Vector3d u = r.axis_direction;

    // ---- phi0: 取两件数据的圆均值反向, 使分支切口落在被测区域之外(§2.1) ----
    CylinderSurfaceFrame probe;
    probe.init(c, u, params_.design_radius, 0.0);
    std::vector<double> phis;
    auto collectPhi = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const std::vector<int>& idx) {
        const int tot = single_mode_ ? static_cast<int>(idx.size())
                                     : static_cast<int>(cloud ? cloud->size() : 0);
        if (tot <= 0) return;
        const int stride = std::max(1, tot / 20000);
        for (int k = 0; k < tot; k += stride) {
            const int i = single_mode_ ? idx[k] : k;
            if (i < 0 || i >= static_cast<int>(cloud->size())) continue;
            const auto& p = cloud->points[i];
            if (!pcl::isFinite(p)) continue;
            double phi = 0.0;
            if (probe.rawPhi(Eigen::Vector3d(p.x, p.y, p.z), phi)) phis.push_back(phi);
        }
    };
    collectPhi(near_cloud_, near_indices_);
    collectPhi(far_cloud_, far_indices_);
    const double phi0 = CylinderSurfaceFrame::choosePhi0(phis);

    if (!frame_.init(c, u, params_.design_radius, phi0)) {
        reason = "参考圆柱坐标系建立失败(轴线或半径非法)";
        return false;
    }

    // ---- 逐点展开 ----
    auto projectAll = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const std::vector<int>& idx,
                          std::vector<int>& src,
                          std::vector<double>& A,
                          std::vector<double>& S,
                          std::vector<double>& E) {
        src.clear(); A.clear(); S.clear(); E.clear();
        if (!cloud) return;
        const int tot = single_mode_ ? static_cast<int>(idx.size())
                                     : static_cast<int>(cloud->size());
        src.reserve(tot);
        for (int k = 0; k < tot; ++k) {
            const int i = single_mode_ ? idx[k] : k;
            if (i < 0 || i >= static_cast<int>(cloud->size())) continue;
            const auto& p = cloud->points[i];
            if (!pcl::isFinite(p)) continue;
            double a = 0, s = 0, e = 0;
            if (!frame_.project(Eigen::Vector3d(p.x, p.y, p.z), a, s, e)) continue;
            if (!finiteD(a) || !finiteD(s) || !finiteD(e)) continue;
            src.push_back(i); A.push_back(a); S.push_back(s); E.push_back(e);
        }
    };
    projectAll(near_cloud_, near_indices_, near_src_, near_a_, near_s_, near_e_);
    projectAll(far_cloud_, far_indices_, far_src_, far_a_, far_s_, far_e_);

    if (near_src_.size() < 50 || far_src_.size() < 200) {
        reason = "展开后有效点不足(近件 " + std::to_string(near_src_.size()) +
                 " 点, 远件 " + std::to_string(far_src_.size()) + " 点)";
        return false;
    }

    // ---- 测量区域(ROI) ----
    if (!roi_set_) {
        double aLo = std::numeric_limits<double>::max(), aHi = -std::numeric_limits<double>::max();
        double sLo = std::numeric_limits<double>::max(), sHi = -std::numeric_limits<double>::max();
        for (double v : near_a_) { aLo = std::min(aLo, v); aHi = std::max(aHi, v); }
        for (double v : far_a_)  { aLo = std::min(aLo, v); aHi = std::max(aHi, v); }
        for (double v : near_s_) { sLo = std::min(sLo, v); sHi = std::max(sHi, v); }
        for (double v : far_s_)  { sLo = std::min(sLo, v); sHi = std::max(sHi, v); }
        roi_a0_ = aLo; roi_a1_ = aHi; roi_s0_ = sLo; roi_s1_ = sHi;
    }
    r.roi_a0 = roi_a0_; r.roi_a1 = roi_a1_; r.roi_s0 = roi_s0_; r.roi_s1 = roi_s1_;

    const double span = roi_s1_ - roi_s0_;
    if (span > 0.90 * kPi * params_.design_radius) {
        reason = "测量区域的周向跨度 " + fmt(span, 1) + " mm 接近半周(piR="
                 + fmt(kPi * params_.design_radius, 1) + " mm), 角度展开分支不可靠; "
                 "请缩小接缝区域, 使跨度远小于半周";
        return false;
    }

    // ---- (a,s,0) 点云 + KD 树: 只装入 ROI 内的点, 并保留 KD 索引 -> 局部索引映射 ----
    // KD 树先建: 真实点距必须由它估计(见 estimatePitchFromKd 的说明)。
    auto buildKd = [&](const std::vector<double>& A, const std::vector<double>& S,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& holder,
                       std::vector<int>& map,
                       pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kd) {
        holder.reset(new pcl::PointCloud<pcl::PointXYZ>);
        map.clear();
        for (size_t k = 0; k < A.size(); ++k) {
            if (A[k] < roi_a0_ || A[k] > roi_a1_ || S[k] < roi_s0_ || S[k] > roi_s1_) continue;
            holder->points.emplace_back(static_cast<float>(A[k]),
                                        static_cast<float>(S[k]), 0.0f);
            map.push_back(static_cast<int>(k));
        }
        holder->width = static_cast<std::uint32_t>(holder->points.size());
        holder->height = 1;
        kd.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        if (!holder->empty()) kd->setInputCloud(holder);
    };
    buildKd(near_a_, near_s_, as_near_holder_, near_kd_map_, near_kd_);
    buildKd(far_a_, far_s_, as_far_holder_, far_kd_map_, far_kd_);

    // ---- 真实点距(栅格与角度空缺邻域半径的唯一依据) ----
    {
        const double pn = estimatePitchFromKd(near_kd_, as_near_holder_);
        const double pf = estimatePitchFromKd(far_kd_, as_far_holder_);
        double p = -1.0;
        if (pn > 0 && pf > 0) p = 0.5 * (pn + pf);
        else if (pn > 0) p = pn;
        else if (pf > 0) p = pf;
        if (p > 0) point_pitch_ = p;
    }

    // ---- 栅格尺度 / 邻域 / 搜索窗口(§4.3 记录实际尺度) ----
    double cell = params_.grid_cell > 0 ? params_.grid_cell
                                        : std::max(2.0 * point_pitch_, 0.5);
    const double roiArea = std::max(1e-9, (roi_a1_ - roi_a0_) * (roi_s1_ - roi_s0_));
    const double density = static_cast<double>(near_src_.size() + far_src_.size()) / roiArea;
    if (density > 0) cell = std::max(cell, std::sqrt(4.0 / density));
    params_.grid_cell = cell;
    params_.edge_neighborhood = params_.edge_neighborhood > 0 ? params_.edge_neighborhood
                                                             : 3.0 * cell;
    params_.search_window = params_.search_window > 0 ? params_.search_window : 10.0 * cell;
    params_.max_gap = params_.max_gap > 0 ? params_.max_gap : 8.0 * cell;
    params_.seam_search_max = params_.seam_search_max > 0
        ? params_.seam_search_max : 10.0 * cell;
    // 细化窗口默认 3×栅格(≈6×点距): 窗口必须明显小于"最小可辨特征宽度",
    // 否则局部窄张口会被局部直线拟合压平(方案 §4.4/§5.3)。见 local_peak 案例。
    params_.edge_smooth_window = params_.edge_smooth_window > 0
        ? params_.edge_smooth_window : 3.0 * cell;
    if (params_.max_intersection_error <= 0) params_.max_intersection_error = 2.0 * cell;
    if (params_.platform_tol <= 0) params_.platform_tol = std::max(cell, 0.01);

    r.grid_cell = cell;
    r.edge_neighborhood = params_.edge_neighborhood;
    r.search_window = params_.search_window;
    r.actual_point_pitch = point_pitch_;

    // ---- 栅格(只做候选搜索与加速, §1.6) ----
    auto buildGrid = [&](const std::vector<double>& A, const std::vector<double>& S, Grid& g) {
        g.cell = cell;
        g.a0 = roi_a0_;
        g.s0 = roi_s0_;
        g.na = std::max(1, static_cast<int>(std::ceil((roi_a1_ - roi_a0_) / cell)) + 1);
        g.ns = std::max(1, static_cast<int>(std::ceil((roi_s1_ - roi_s0_) / cell)) + 1);
        const size_t ncell = static_cast<size_t>(g.na) * static_cast<size_t>(g.ns);
        g.occupied.assign(ncell, 0);
        g.buckets.assign(ncell, std::vector<int>());
        for (size_t k = 0; k < A.size(); ++k) {
            int ia = 0, is = 0;
            g.cellOf(A[k], S[k], ia, is);
            if (!g.inside(ia, is)) continue;
            g.occupied[g.idx(ia, is)] = 1;
            g.buckets[g.idx(ia, is)].push_back(static_cast<int>(k));
        }
    };
    buildGrid(near_a_, near_s_, near_grid_);
    buildGrid(far_a_, far_s_, far_grid_);

    return true;
}

int MeasureWeldPreparation::nearestOtherPoint(bool other_is_near, double a, double s,
                                              double& dist) const
{
    dist = std::numeric_limits<double>::max();
    const auto& kd = other_is_near ? near_kd_ : far_kd_;
    const auto& map = other_is_near ? near_kd_map_ : far_kd_map_;
    const auto& holder = other_is_near ? as_near_holder_ : as_far_holder_;
    if (!kd || !holder || holder->empty()) return -1;
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    const pcl::PointXYZ q(static_cast<float>(a), static_cast<float>(s), 0.0f);
    if (kd->nearestKSearch(q, 1, idx, d2) < 1) return -1;
    dist = std::sqrt(static_cast<double>(d2[0]));
    if (idx[0] < 0 || idx[0] >= static_cast<int>(map.size())) return -1;
    return map[idx[0]];
}

// ============================================================
// 点距估计: 在 ROI 的 (a,s) KD 树上随机抽样查"第 2 近邻"。
// ============================================================
double MeasureWeldPreparation::estimatePitchFromKd(
    const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kd,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& holder) const
{
    if (!kd || !holder) return -1.0;
    const int n = static_cast<int>(holder->size());
    if (n < 20) return -1.0;
    const int K = std::min(800, n / 2);
    std::mt19937 rng(20260926u);
    std::uniform_int_distribution<int> pick(0, n - 1);
    std::vector<int> idx(2);
    std::vector<float> d2(2);
    std::vector<double> ds;
    ds.reserve(K);
    for (int k = 0; k < K; ++k) {
        const int i = pick(rng);
        if (kd->nearestKSearch(holder->points[i], 2, idx, d2) >= 2) {
            ds.push_back(std::sqrt(static_cast<double>(d2[1])));
        }
    }
    if (ds.empty()) return -1.0;
    std::sort(ds.begin(), ds.end());
    const double med = ds[ds.size() / 2];
    return (med > 1e-6) ? med : -1.0;
}

// ============================================================
// 同一侧多段边缘的共线性检查(区分"同一条缝的断点"与"多条接缝候选")
// ============================================================
bool MeasureWeldPreparation::edgeSegmentsCompatible(const Edge& e, std::string& why) const
{
    if (e.polylines.size() <= 1) return true;
    struct Seg { Eigen::Vector2d d; Eigen::Vector2d c; };
    std::vector<Seg> segs;
    for (const auto& sgm : e.polylines) {
        if (sgm.size() < 2) continue;
        Eigen::Vector2d m = Eigen::Vector2d::Zero();
        for (int k : sgm) m += Eigen::Vector2d(e.points[k].a, e.points[k].s);
        m /= static_cast<double>(sgm.size());
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (int k : sgm) {
            const Eigen::Vector2d d = Eigen::Vector2d(e.points[k].a, e.points[k].s) - m;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        if (es.info() != Eigen::Success) continue;
        Seg sg;
        sg.d = Eigen::Vector2d(es.eigenvectors().col(1)).normalized();
        sg.c = m;
        segs.push_back(sg);
    }
    if (segs.size() <= 1) return true;

    const double angTol = 5.0 * kPi / 180.0;
    const double offTol = std::max(3.0 * params_.grid_cell, 4.0 * point_pitch_);
    const Eigen::Vector2d nrm(-segs[0].d.y(), segs[0].d.x());
    for (size_t i = 1; i < segs.size(); ++i) {
        const double ang = std::acos(std::min(1.0, std::fabs(segs[0].d.dot(segs[i].d))));
        if (ang > angTol) {
            why = "相邻边缘段的走向不一致(夹角 " + fmt(ang * 180.0 / kPi, 1)
                + "° > " + fmt(angTol * 180.0 / kPi, 1) + "°)";
            return false;
        }
        const double off = std::fabs((segs[i].c - segs[0].c).dot(nrm));
        if (off > offTol) {
            why = "相邻边缘段互相平行但垂距 " + fmt(off, 2) + " mm 超过 "
                + fmt(offTol, 2) + " mm, 属于不同的接缝而不是同一条缝的断点";
            return false;
        }
    }
    return true;
}

// ============================================================
// 邻域角度空缺判据(方案 §4.4 两级方法的第二级)
// 栅格只挑出候选; 判据回到**原始展开点**, 邻域半径按点距取, 因此边缘点位置
// 直接落在真实采样点上(误差 ~ 点距), 而不是被量化到一个栅格边长。
// ============================================================
bool MeasureWeldPreparation::supportByAngularGap(bool from_near, double a, double s,
                                                 double& support, double& outward_rad,
                                                 int& neighbor_count) const
{
    support = 0.0;
    outward_rad = 0.0;
    neighbor_count = 0;
    const auto& kd = from_near ? near_kd_ : far_kd_;
    const auto& holder = from_near ? as_near_holder_ : as_far_holder_;
    if (!kd || !holder || holder->empty()) return false;

    const double rMin = std::max(1e-6, params_.radius_min_factor * point_pitch_);
    const double rMax = std::max(rMin * 1.5, params_.radius_max_factor * point_pitch_);

    std::vector<int> idx;
    std::vector<float> d2;
    const pcl::PointXYZ q(static_cast<float>(a), static_cast<float>(s), 0.0f);
    const int found = kd->radiusSearch(q, static_cast<float>(rMax), idx, d2);
    if (found < 3) {
        neighbor_count = found;
        return false;   // 邻域点太少, 无法判定角度空缺
    }

    std::vector<double> ang;
    ang.reserve(found);
    for (int k = 0; k < found; ++k) {
        const double d = std::sqrt(static_cast<double>(d2[k]));
        if (d < rMin) continue;                    // 剔除自身
        const auto& pt = holder->points[idx[k]];
        const double da = static_cast<double>(pt.x) - a;
        const double ds = frame_.arcDifference(static_cast<double>(pt.y), s);
        if (std::fabs(da) < 1e-9 && std::fabs(ds) < 1e-9) continue;
        double th = std::atan2(ds, da);
        if (th < 0.0) th += 2.0 * kPi;
        ang.push_back(th);
    }
    neighbor_count = static_cast<int>(ang.size());
    if (ang.size() < 3) return false;
    std::sort(ang.begin(), ang.end());

    double maxGap = 0.0, gapStart = 0.0;
    for (size_t k = 0; k + 1 < ang.size(); ++k) {
        const double g = ang[k + 1] - ang[k];
        if (g > maxGap) { maxGap = g; gapStart = ang[k]; }
    }
    const double wrapGap = ang.front() + 2.0 * kPi - ang.back();
    if (wrapGap > maxGap) { maxGap = wrapGap; gapStart = ang.back(); }

    support = 1.0 - maxGap / (2.0 * kPi);
    outward_rad = gapStart + 0.5 * maxGap;
    while (outward_rad >= 2.0 * kPi) outward_rad -= 2.0 * kPi;
    return true;
}

// ============================================================
// §4.4 提取接缝侧真实边缘
// ============================================================
bool MeasureWeldPreparation::buildEdgeFor(bool from_near, Edge& out, const std::string& tag)
{
    const std::vector<double>& A = from_near ? near_a_ : far_a_;
    const std::vector<double>& S = from_near ? near_s_ : far_s_;
    const std::vector<double>& E = from_near ? near_e_ : far_e_;
    const Grid& g = from_near ? near_grid_ : far_grid_;

    out = Edge();
    if (A.empty()) {
        out.note = tag + ": 展开点为空";
        return false;
    }

    const double cell = g.cell;
    const double facingCos = std::cos(params_.facing_tol_deg * kPi / 180.0);

    // ---- 1. 占据图找边界候选格 ----
    std::vector<int> candCells;
    for (int ia = 0; ia < g.na; ++ia) {
        for (int is = 0; is < g.ns; ++is) {
            if (!g.occ(ia, is)) continue;
            bool boundary = false;
            const int da[4] = { 1, -1, 0, 0 };
            const int ds[4] = { 0, 0, 1, -1 };
            for (int k = 0; k < 4; ++k) {
                if (!g.occ(ia + da[k], is + ds[k])) { boundary = true; break; }
            }
            if (boundary) candCells.push_back(g.idx(ia, is));
        }
    }

    // ---- 2. 回到候选格邻近的原始展开点, 做邻域角度空缺判定 ----
    for (int cIdx : candCells) {
        const int ia0 = cIdx / g.ns;
        const int is0 = cIdx % g.ns;
        const bool touchRoi = (ia0 <= 0 || is0 <= 0 || ia0 >= g.na - 1 || is0 >= g.ns - 1);
        for (int local : g.buckets[cIdx]) {
            EdgePoint ep;
            ep.source_index = from_near ? near_src_[local] : far_src_[local];
            ep.from_near = from_near;
            ep.a = A[local];
            ep.s = S[local];
            ep.e = E[local];
            ep.xyz = frame_.unproject(ep.a, ep.s, ep.e);

            double outwardRad = 0.0, supportVal = 0.0;
            int neighborCount = 0;
            const bool gapOk = supportByAngularGap(from_near, ep.a, ep.s,
                                                   supportVal, outwardRad, neighborCount);
            ep.support = supportVal;
            double outwardDeg = outwardRad * 180.0 / kPi;
            while (outwardDeg >= 360.0) outwardDeg -= 360.0;
            while (outwardDeg < 0.0) outwardDeg += 360.0;
            ep.outward_deg = outwardDeg;
            const double maxGapDeg = (1.0 - supportVal) * 360.0;
            const bool isBoundary = gapOk && (maxGapDeg >= params_.boundary_gap_deg);
            const double mid = outwardRad;

            double dist = std::numeric_limits<double>::max();
            const int oi = nearestOtherPoint(!from_near, ep.a, ep.s, dist);
            ep.to_other_mm = dist;
            const bool otherNearby = std::isfinite(dist) && (dist <= params_.seam_search_max);

            bool facing = false;
            if (otherNearby && oi >= 0) {
                const auto& otherA = from_near ? far_a_ : near_a_;
                const auto& otherS = from_near ? far_s_ : near_s_;
                if (oi < static_cast<int>(otherA.size())) {
                    const double da = otherA[oi] - ep.a;
                    const double ds = frame_.arcDifference(otherS[oi], ep.s);
                    const double toward = std::atan2(ds, da);
                    facing = std::cos(mid - toward) >= facingCos;
                }
            }

            ep.touch_roi = touchRoi;
            if (!gapOk) {
                ep.valid = false;
                ep.reject_reason = "邻域点过少(仅 " + std::to_string(neighborCount)
                    + " 点), 无法判定角度空缺: 更像孤立尖刺或扫描拖尾";
            }
            else if (!isBoundary) {
                ep.valid = false;
                ep.reject_reason = "邻域角度空缺仅 " + fmt(maxGapDeg, 1)
                    + "°(< " + fmt(params_.boundary_gap_deg, 0) + "°): 本侧表面完整, 不是边缘";
            }
            else if (ep.support < params_.min_edge_support) {
                ep.valid = false;
                ep.reject_reason = "本侧表面支持不足(支持率 " + fmt(ep.support, 2) + ")";
            }
            else if (!otherNearby) {
                ep.valid = false;
                ep.reject_reason = "附近没有对方试样(最近距离 "
                    + (std::isfinite(dist) ? fmt(dist, 1) : std::string("超出范围"))
                    + " mm > " + fmt(params_.seam_search_max, 1)
                    + " mm): 更像外部视界边界/孔洞/裁剪边界";
            }
            else if (!facing) {
                ep.valid = false;
                ep.reject_reason = "空缺方向未朝向对方试样(朝向 " + fmt(ep.outward_deg, 0)
                    + "°): 非接缝侧边界";
            }
            else {
                ep.valid = true;
                // 外推到材料"真实边界": 邻域半径 r 内的角度空缺 void=(1-support)*360°;
                // 对直边界有 d = r*cos(void/2)(在边界上 void=180°->d=0, 退到 r 时 void=0->d=r)。
                // 不做的后果: 候选边缘点是一条 0~2mm 宽的带(材料内侧), "取最大"会挑中退得最深
                // 的那个点, 使间隙系统性偏大约一格(实测 +0.9mm @格=0.83mm)。
                {
                    const double rMinE = std::max(1e-6, params_.radius_min_factor * point_pitch_);
                    const double rMaxE = std::max(rMinE * 1.5, params_.radius_max_factor * point_pitch_);
                    const double voidRad = (1.0 - supportVal) * 2.0 * kPi;
                    const double dEdge = rMaxE * std::cos(0.5 * voidRad);
                    if (dEdge > 0.0 && dEdge < rMaxE) {
                        ep.a += dEdge * std::cos(outwardRad);
                        ep.s += dEdge * std::sin(outwardRad);
                        ep.xyz = frame_.unproject(ep.a, ep.s, ep.e);
                    }
                }
            }
            out.points.push_back(std::move(ep));
        }
    }
    out.candidate_count = static_cast<int>(out.points.size());
    if (out.points.empty()) {
        out.note = tag + ": 未找到任何接缝侧边缘候选";
        return false;
    }

    // ---- 3. 有效点排序 + 分段(保留真实断点) ----
    std::vector<int> validIdx;
    for (int k = 0; k < static_cast<int>(out.points.size()); ++k) {
        if (out.points[k].valid) validIdx.push_back(k);
    }
    out.valid_count = static_cast<int>(validIdx.size());
    if (validIdx.size() < static_cast<size_t>(params_.min_edge_points)) {
        out.note = tag + ": 通过筛选的边缘点仅 " + std::to_string(validIdx.size())
            + " 个(少于 " + std::to_string(params_.min_edge_points) + "), 视为无有效边缘";
        return false;
    }

    // 接缝走向: 对有效点做 PCA
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (int k : validIdx) mean += Eigen::Vector2d(out.points[k].a, out.points[k].s);
    mean /= static_cast<double>(validIdx.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (int k : validIdx) {
        const Eigen::Vector2d d = Eigen::Vector2d(out.points[k].a, out.points[k].s) - mean;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    if (es.info() != Eigen::Success) {
        out.note = tag + ": 边缘主方向估计失败";
        return false;
    }
    const Eigen::Vector2d tangent = Eigen::Vector2d(es.eigenvectors().col(1)).normalized();

    std::sort(validIdx.begin(), validIdx.end(), [&](int lhs, int rhs) {
        const double pl = out.points[lhs].a * tangent.x() + out.points[lhs].s * tangent.y();
        const double pr = out.points[rhs].a * tangent.x() + out.points[rhs].s * tangent.y();
        return pl < pr;
    });

    std::vector<std::vector<int>> segs;
    {
        std::vector<int> cur;
        for (size_t k = 0; k < validIdx.size(); ++k) {
            if (!cur.empty()) {
                const auto& pp = out.points[cur.back()];
                const auto& p = out.points[validIdx[k]];
                const double da = p.a - pp.a;
                const double ds = frame_.arcDifference(p.s, pp.s);
                const double step = std::sqrt(da * da + ds * ds);
                if (step > params_.max_gap) {
                    segs.push_back(cur);
                    cur.clear();
                }
            }
            cur.push_back(validIdx[k]);
        }
        if (!cur.empty()) segs.push_back(cur);
    }
    for (auto& sgm : segs) {
        if (static_cast<int>(sgm.size()) >= params_.min_edge_points) out.polylines.push_back(std::move(sgm));
    }

    {
        double aLo = std::numeric_limits<double>::max(), aHi = -std::numeric_limits<double>::max();
        double sLo = std::numeric_limits<double>::max(), sHi = -std::numeric_limits<double>::max();
        for (int k : validIdx) {
            aLo = std::min(aLo, out.points[k].a); aHi = std::max(aHi, out.points[k].a);
            sLo = std::min(sLo, out.points[k].s); sHi = std::max(sHi, out.points[k].s);
        }
        out.a_min = aLo; out.a_max = aHi; out.s_min = sLo; out.s_max = sHi;
    }

    if (out.polylines.empty()) {
        out.note = tag + ": 有效边缘点无法连成连续段(断点过多)";
        return false;
    }

    // ---- 4. 局部稳健直线细化(只在有效观测段内插值, 不外推) ----
    out.refined.resize(out.polylines.size());
    out.refined_sigma.assign(out.polylines.size(), 0.0);
    for (size_t sgi = 0; sgi < out.polylines.size(); ++sgi) {
        const auto& sgm = out.polylines[sgi];
        const int n = static_cast<int>(sgm.size());
        double spacing = cell;
        if (n > 1) {
            double sum = 0.0;
            for (int k = 1; k < n; ++k) {
                const auto& p0 = out.points[sgm[k - 1]];
                const auto& p1 = out.points[sgm[k]];
                const double da = p1.a - p0.a;
                const double ds = frame_.arcDifference(p1.s, p0.s);
                sum += std::sqrt(da * da + ds * ds);
            }
            spacing = std::max(1e-6, sum / (n - 1));
        }
        int halfWin = static_cast<int>(std::lround(params_.edge_smooth_window / spacing / 2.0));
        halfWin = std::max(2, std::min(halfWin, std::max(2, n / 2)));

        std::vector<Eigen::Vector2d> refined;
        refined.reserve(n);
        double sigmaMax = 0.0;
        for (int k = 0; k < n; ++k) {
            const int lo = std::max(0, k - halfWin);
            const int hi = std::min(n - 1, k + halfWin);
            std::vector<Eigen::Vector2d> win;
            win.reserve(hi - lo + 1);
            for (int j = lo; j <= hi; ++j) {
                win.emplace_back(out.points[sgm[j]].a, out.points[sgm[j]].s);
            }
            const Line2D L = fitLine2DRobust(win);
            const Eigen::Vector2d p(out.points[sgm[k]].a, out.points[sgm[k]].s);
            if (L.ok) {
                const double t = (p - L.p).dot(L.d);
                refined.push_back(L.p + t * L.d);
                sigmaMax = std::max(sigmaMax, L.sigma);
            }
            else {
                refined.push_back(p);
            }
        }
        out.refined[sgi] = std::move(refined);
        out.refined_sigma[sgi] = sigmaMax;
    }

    out.ok = true;
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(1);
        oss << tag << ": 候选 " << out.candidate_count << " 点, 有效 " << out.valid_count
            << " 点, 连续边缘段 " << out.polylines.size() << " 条";
        if (out.polylines.size() > 1) oss << "(第一版只支持单条连续接缝)";
        out.note = oss.str();
    }
    return true;
}

bool MeasureWeldPreparation::buildEdges(Result& r, std::string& reason)
{
    if (!reportProgress(3, kStageTotal, "焊前测量: 提取接缝边缘...")) {
        reason = "已取消";
        return false;
    }
    const bool nearOk = buildEdgeFor(true, r.near_edge, "近件接缝边缘");
    if (cancelled_) { reason = "已取消"; return false; }
    const bool farOk = buildEdgeFor(false, r.far_edge, "远件接缝边缘");

    if (!nearOk || !farOk) {
        reason = "接缝边缘无观测支持: " + r.near_edge.note + "; " + r.far_edge.note
            + "。远件的实际接缝边缘必须有观测支持; 无限延伸的拟合曲面不能推断被遮住的板边。";
        return false;
    }
    // 多段边缘: 只有"同一条缝的断点"(互相共线/垂距接近)才允许继续;
    // 否则区域内有多条接缝候选, 第一版要求先缩小 ROI 选定目标缝(§4.4)。
    // 断点本身不会被跨接或外推: 配不到对应点的区间会作为缺测区间报告(§4.5)。
    std::string whyNear, whyFar;
    if (!edgeSegmentsCompatible(r.near_edge, whyNear)) {
        reason = "近件接缝边缘存在多条互不相同的接缝候选(" +
            std::to_string(r.near_edge.polylines.size()) + " 段): " + whyNear +
            "。第一版只支持单条、无分叉、直线或缓弯接缝, 请缩小测量区域(ROI)只保留目标接缝。";
        return false;
    }
    if (!edgeSegmentsCompatible(r.far_edge, whyFar)) {
        reason = "远件接缝边缘存在多条互不相同的接缝候选(" +
            std::to_string(r.far_edge.polylines.size()) + " 段): " + whyFar +
            "。第一版只支持单条、无分叉、直线或缓弯接缝, 请缩小测量区域(ROI)只保留目标接缝。";
        return false;
    }
    return true;
}

// ============================================================
// §4.5 沿指定方向配对
// ============================================================
int MeasureWeldPreparation::intersectRayWithPolylines(
    const Eigen::Vector2d& origin, const Eigen::Vector2d& q,
    const std::vector<std::vector<Eigen::Vector2d>>& polylines,
    const std::vector<double>& sigmas,
    double maxLambda, bool bothSides,
    double parallel_ratio_min, double max_amp,
    std::vector<RayHit>& hits, int& parallel_rejects, int& amp_rejects) const
{
    hits.clear();
    parallel_rejects = 0;
    amp_rejects = 0;
    for (size_t sgi = 0; sgi < polylines.size(); ++sgi) {
        const auto& poly = polylines[sgi];
        const double sigma = (sgi < sigmas.size()) ? sigmas[sgi] : 0.0;
        for (size_t k = 0; k + 1 < poly.size(); ++k) {
            double t = 0.0, ratio = 1.0;
            if (!raySegment(origin, q, poly[k], poly[k + 1], t, ratio)) continue;
            if (bothSides) {
                if (std::fabs(t) > maxLambda) continue;
            }
            else {
                if (t < 0.0 || t > maxLambda) continue;
            }
            RayHit h;
            h.lambda = t;
            h.seg = static_cast<int>(sgi);
            h.px = origin.x() + t * q.x();
            h.py = origin.y() + t * q.y();
            h.ratio = ratio;
            h.sigma = sigma;
            h.amp = (ratio > 1e-12) ? sigma / ratio : std::numeric_limits<double>::max();
            if (ratio < parallel_ratio_min) {
                h.rejected = true;
                ++parallel_rejects;
            }
            else if (h.amp > max_amp) {
                h.rejected = true;
                ++amp_rejects;
            }
            hits.push_back(h);
        }
    }
    std::vector<double> good;
    for (const auto& h : hits) if (!h.rejected) good.push_back(h.lambda);
    if (good.empty()) return 0;
    std::sort(good.begin(), good.end());
    const double tol = std::max(2.0 * point_pitch_, params_.grid_cell);
    int clusters = 1;
    for (size_t k = 1; k < good.size(); ++k) {
        if (good[k] - good[k - 1] > tol) ++clusters;
    }
    return clusters;
}

bool MeasureWeldPreparation::pairAlongDirection(Result& r, std::string& reason)
{
    if (!reportProgress(4, kStageTotal, "焊前测量: 配对...")) {
        reason = "已取消";
        return false;
    }

    const auto& nearEdge = r.near_edge;
    const auto& farEdge = r.far_edge;
    const auto& farLines = farEdge.refined;

    // ---- 近件全部有效边缘点: 沿共同接缝走向统一排序(段内顺序已由边缘提取保证) ----
    std::vector<int>         nearAll;   // near_edge.points 的下标
    std::vector<int>         nearSeg;   // 所属折线段号
    std::vector<int>         blockLo, blockHi;   // 每点所属段在 nearAll 中的下标区间
    Eigen::Vector2d          seamDir(1.0, 0.0);
    {
        struct Item { double t; int pt; int seg; };
        std::vector<Item> items;
        // 共同走向 = 各段主方向的平均(段内已按各自主方向排序)
        Eigen::Vector2d dirSum = Eigen::Vector2d::Zero();
        for (size_t si = 0; si < nearEdge.polylines.size(); ++si) {
            const auto& sgm = nearEdge.polylines[si];
            if (sgm.size() < 2) continue;
            Eigen::Vector2d m = Eigen::Vector2d::Zero();
            for (int k : sgm) m += Eigen::Vector2d(nearEdge.points[k].a, nearEdge.points[k].s);
            m /= static_cast<double>(sgm.size());
            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (int k : sgm) {
                const Eigen::Vector2d d =
                    Eigen::Vector2d(nearEdge.points[k].a, nearEdge.points[k].s) - m;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
            if (es.info() != Eigen::Success) continue;
            Eigen::Vector2d dd = Eigen::Vector2d(es.eigenvectors().col(1)).normalized();
            if (dirSum.norm() > 1e-9 && dd.dot(dirSum) < 0) dd = -dd;   // 统一朝向
            dirSum += dd;
            for (int k : sgm) {
                Item it;
                it.pt = k;
                it.seg = static_cast<int>(si);
                it.t = nearEdge.points[k].a;      // 占位, 排序前重算
                items.push_back(it);
            }
        }
        if (dirSum.norm() > 1e-9) seamDir = dirSum.normalized();
        for (auto& it : items) {
            it.t = nearEdge.points[it.pt].a * seamDir.x() + nearEdge.points[it.pt].s * seamDir.y();
        }
        std::stable_sort(items.begin(), items.end(),
                         [](const Item& x, const Item& y) { return x.t < y.t; });
        nearAll.reserve(items.size());
        nearSeg.reserve(items.size());
        blockLo.assign(items.size(), 0);
        blockHi.assign(items.size(), 0);
        for (const auto& it : items) { nearAll.push_back(it.pt); nearSeg.push_back(it.seg); }
        size_t i = 0;
        while (i < nearAll.size()) {
            size_t j = i;
            while (j + 1 < nearAll.size() && nearSeg[j + 1] == nearSeg[i]) ++j;
            for (size_t k = i; k <= j; ++k) { blockLo[k] = static_cast<int>(i); blockHi[k] = static_cast<int>(j); }
            i = j + 1;
        }
    }
    if (nearAll.empty()) {
        reason = "近件边缘没有可用于配对的点";
        return false;
    }

    // ---- 方向定义(口径统一 2026-09: 配对方向 = 跨缝方向) ----
    //   口径: 间隙 = 沿跨缝方向的开口宽度(环缝 -> 轴向); 阶差 = 径向 e_P - e_Q。
    //   两个量由同一次配对得到, 所以配对方向只由 gap_direction 决定, 与 metric 无关。
    Eigen::Vector2d qGlobal(1.0, 0.0);
    bool qPerPoint = false;
    if (params_.gap_direction == GapDirection::Axial) {
        double na = 0.0, fa = 0.0;
        int nc = 0, fc = 0;
        for (int k : nearAll) { na += nearEdge.points[k].a; ++nc; }
        for (size_t si = 0; si < farEdge.polylines.size(); ++si)
            for (int k : farEdge.polylines[si]) { fa += farEdge.points[k].a; ++fc; }
        na = nc ? na / nc : 0.0;
        fa = fc ? fa / fc : 0.0;
        const double sgn = (fa >= na) ? 1.0 : -1.0;
        qGlobal = Eigen::Vector2d(sgn, 0.0);   // 等周向坐标 s=常数, 沿轴向配
    }
    else {
        qPerPoint = true;                      // 曲面内接缝法向(逐点)
    }

    if (qPerPoint) {
        r.direction_label =
            "曲面内接缝法向(由两条接缝边缘的中心走向定切向, q 与其垂直并朝向远件)";
    }
    else {
        r.direction_label = std::string("沿圆柱轴向(")
            + (qGlobal.x() > 0 ? "+u 方向" : "-u 方向")
            + "), 间隙 = 该方向上的开口宽度(环缝用本模式)";
    }

    // ---- seam-normal 模式: 先由两条边缘建立中心走向(窗口不跨段) ----
    std::vector<Eigen::Vector2d> centerPts(nearAll.size());
    std::vector<Eigen::Vector2d> centerToFar(nearAll.size());
    if (qPerPoint) {
        for (size_t k = 0; k < nearAll.size(); ++k) {
            const auto& p = nearEdge.points[nearAll[k]];
            double d = 0.0;
            const int fi = nearestOtherPoint(false, p.a, p.s, d);
            Eigen::Vector2d farPt;
            if (fi >= 0 && fi < static_cast<int>(far_a_.size())) {
                farPt = Eigen::Vector2d(far_a_[fi], far_s_[fi]);
            }
            else {
                farPt = Eigen::Vector2d(p.a, p.s) + params_.search_window * qGlobal;
            }
            centerPts[k] = 0.5 * (Eigen::Vector2d(p.a, p.s) + farPt);
            centerToFar[k] = farPt - Eigen::Vector2d(p.a, p.s);
        }
    }

    const double maxAmp = params_.max_intersection_error;
    std::vector<double> seamT;
    seamT.reserve(nearAll.size());
    r.matches.reserve(nearAll.size());

    int parallelRej = 0, ampRej = 0, noHit = 0, multiHit = 0, widenHits = 0;

    for (size_t k = 0; k < nearAll.size(); ++k) {
        const EdgePoint& P = nearEdge.points[nearAll[k]];
        Match m;
        m.near_index = P.source_index;
        m.near_a = P.a;
        m.near_s = P.s;
        m.near_xyz = P.xyz;
        m.near_proj_xyz = frame_.unproject(P.a, P.s, 0.0);
        seamT.push_back(P.a * seamDir.x() + P.s * seamDir.y());

        Eigen::Vector2d q = qGlobal;
        if (qPerPoint) {
            // 局部切向只在同一段内取窗口, 不跨越真实断点
            const int lo = std::max(blockLo[k], static_cast<int>(k) - 6);
            const int hi = std::min(blockHi[k], static_cast<int>(k) + 6);
            bool haveN = false;
            if (hi > lo) {
                const Line2D L = fitLine2DRobust(std::vector<Eigen::Vector2d>(
                    centerPts.begin() + lo, centerPts.begin() + hi + 1));
                if (L.ok) {
                    Eigen::Vector2d nrm(-L.d.y(), L.d.x());
                    if (nrm.dot(centerToFar[k]) < 0) nrm = -nrm;
                    if (nrm.norm() > 1e-9) { q = nrm.normalized(); haveN = true; }
                }
            }
            if (!haveN && centerToFar[k].norm() > 1e-9) {
                q = centerToFar[k].normalized();
                haveN = true;
            }
            if (!haveN) {
                m.valid = false;
                m.invalid_reason = "接缝法向不可靠(中心走向退化)";
                r.matches.push_back(std::move(m));
                continue;
            }
        }

        std::vector<RayHit> hits;
        int prej = 0, arej = 0;
        // q 的方向在轴向模式下由"近边平均 a -> 远边平均 a"定向, 法向模式逐点朝向远件,
        // 因此只朝前搜索(命中方向唯一); 找不到就是找不到, 如实记为无交点。
        int clusters = intersectRayWithPolylines(
            Eigen::Vector2d(P.a, P.s), q, farLines, farEdge.refined_sigma,
            params_.search_window, /*bothSides=*/false,
            params_.parallel_ratio_min, maxAmp, hits, prej, arej);
        parallelRej += prej;
        ampRej += arej;

        // ---- 扩窗重试(必须): 先按搜索窗口(默认 10×栅格)找; 射线完全没碰到远边时,
        //      放大到 3 倍窗口(默认 30×栅格)再找一次。真实间隙大于默认窗口时不会再误报不可测。
        //      注意: 只有"根本没交点"才重试; 交点被平行/误差放大剔除属于方向退化, 放大窗口无益。
        double searchedW = params_.search_window;
        if (clusters == 0 && hits.empty()) {
            searchedW = 3.0 * params_.search_window;
            std::vector<RayHit> hitsWide;
            int prejW = 0, arejW = 0;
            const int cW = intersectRayWithPolylines(
                Eigen::Vector2d(P.a, P.s), q, farLines, farEdge.refined_sigma,
                searchedW, /*bothSides=*/false,
                params_.parallel_ratio_min, maxAmp, hitsWide, prejW, arejW);
            parallelRej += prejW;
            ampRej += arejW;
            if (cW > 0) {
                hits.swap(hitsWide);
                clusters = cW;
                ++widenHits;
                m.widened = true;      // 记录: 该配对来自扩窗搜索, 结果需人工确认
            }
        }

        if (clusters == 0) {
            m.valid = false;
            if (prej > 0) {
                m.invalid_reason = "沿该方向与远件边缘近乎平行, 交点误差被放大(退化), 局部不可测";
            }
            else if (arej > 0) {
                m.invalid_reason = "边缘位置不确定性过大(交点误差放大超过 "
                    + fmt(maxAmp, 3) + " mm), 局部不可测";
            }
            else {
                m.invalid_reason = "沿该方向在搜索范围(" + fmt(searchedW, 1)
                    + " mm)内找不到远件边缘交点";
            }
            ++noHit;
            r.matches.push_back(std::move(m));
            continue;
        }
        if (clusters > 1) {
            m.valid = false;
            m.invalid_reason = "沿该方向存在多个远件边缘交点(歧义), 局部不可测";
            ++multiHit;
            r.matches.push_back(std::move(m));
            continue;
        }

        // 唯一簇: 取该簇内 |λ| 最小的未剔除交点
        const double tol = std::max(2.0 * point_pitch_, params_.grid_cell);
        double clusterRef = 0.0;
        bool haveRef = false;
        for (const auto& h : hits) {
            if (!h.rejected) { clusterRef = h.lambda; haveRef = true; break; }
        }
        const RayHit* best = nullptr;
        double bestAbs = std::numeric_limits<double>::max();
        for (const auto& h : hits) {
            if (h.rejected) continue;
            if (haveRef && std::fabs(h.lambda - clusterRef) > tol) continue;
            if (std::fabs(h.lambda) < bestAbs) { bestAbs = std::fabs(h.lambda); best = &h; }
        }
        if (!best) {
            m.valid = false;
            m.invalid_reason = "交点候选全部被剔除(平行退化或误差放大)";
            ++noHit;
            r.matches.push_back(std::move(m));
            continue;
        }

        m.valid = true;
        m.far_seg = best->seg;
        m.far_a = best->px;
        m.far_s = best->py;
        m.lambda = best->lambda;                  // 间隙 = 沿跨缝方向 q 的有符号交点距离
        m.distance = std::fabs(m.lambda);
        m.far_xyz = frame_.unproject(m.far_a, m.far_s, 0.0);

        // 最近的原始远边点(可逆追溯), 并取它的径向残差 e_Q 用于径向阶差
        {
            double bestD = std::numeric_limits<double>::max();
            for (size_t si = 0; si < farEdge.polylines.size(); ++si) {
                for (int fi : farEdge.polylines[si]) {
                    const auto& fp = farEdge.points[fi];
                    const double da = fp.a - m.far_a;
                    const double ds = frame_.arcDifference(fp.s, m.far_s);
                    const double d2 = da * da + ds * ds;
                    if (d2 < bestD) { bestD = d2; m.far_point_index = fp.source_index; m.far_e = fp.e; }
                }
            }
        }

        // 径向阶差(口径: 近件在外为正) = e_P - e_Q, 与间隙同一次配对得到
        m.near_e = P.e;
        m.step_signed = m.near_e - m.far_e;
        m.step_abs = std::fabs(m.step_signed);

        r.matches.push_back(std::move(m));
    }

    // ---- 有效/无效/有效长度/缺测区间 ----
    r.widen_retry_hits = widenHits;   // 其中多少点是靠扩窗重试才配上的
    r.valid_count = 0;
    r.invalid_count = 0;
    for (const auto& m : r.matches) (m.valid ? r.valid_count : r.invalid_count)++;
    {
        double tLo = std::numeric_limits<double>::max(), tHi = -std::numeric_limits<double>::max();
        for (size_t k = 0; k < r.matches.size(); ++k) {
            if (!r.matches[k].valid) continue;
            tLo = std::min(tLo, seamT[k]);
            tHi = std::max(tHi, seamT[k]);
        }
        r.valid_length = (tHi > tLo) ? (tHi - tLo) : 0.0;
    }
    {
        // 近件边缘自身的真实断点: 断点两侧块之间存在一段"没有近件边缘点"的区间,
        // 这段同样不可测, 必须作为缺测区间报告(否则会误称已确定整条缝的全局最大值)。
        const double gapMin0 = std::max(2.5 * point_pitch_, 0.5 * params_.grid_cell);
        size_t i = 0;
        while (i < nearAll.size()) {
            const size_t j = static_cast<size_t>(blockHi[i]);
            if (j + 1 < nearAll.size()) {
                const double t0 = seamT[j];
                const double t1 = seamT[j + 1];
                if (t1 - t0 > gapMin0) {
                    r.missing_intervals.push_back("[" + fmt(t0, 1) + ", " + fmt(t1, 1)
                        + "] mm(沿接缝走向), 长度 " + fmt(t1 - t0, 1)
                        + " mm(近件接缝边缘在此断开)");
                }
            }
            i = j + 1;
        }
    }
    {
        // 缺测区间 = 有效测量段内连续配不到对应点的区间(断点不会被跨接或外推)
        int firstValid = -1, lastValid = -1;
        for (size_t k = 0; k < r.matches.size(); ++k) {
            if (r.matches[k].valid) { if (firstValid < 0) firstValid = static_cast<int>(k); lastValid = static_cast<int>(k); }
        }
        bool inGap = false;
        double gapStart = 0.0;
        const double gapMin = std::max(2.5 * point_pitch_, 0.5 * params_.grid_cell);
        for (int k = firstValid; k >= 0 && k <= lastValid; ++k) {
            const bool bad = !r.matches[k].valid;
            if (bad && !inGap) { inGap = true; gapStart = seamT[k]; }
            if ((!bad || k == lastValid) && inGap) {
                const double gapEnd = seamT[k];
                inGap = false;
                if (gapEnd - gapStart > gapMin) {
                    r.missing_intervals.push_back("[" + fmt(gapStart, 1) + ", "
                        + fmt(gapEnd, 1) + "] mm(沿接缝走向), 长度 "
                        + fmt(gapEnd - gapStart, 1) + " mm");
                }
            }
        }
    }
    {
        std::unordered_map<std::string, int> cnt;
        for (const auto& m : r.matches) if (!m.valid) cnt[m.invalid_reason]++;
        for (const auto& kv : cnt) {
            r.invalid_reasons.push_back(kv.first + " —— " + std::to_string(kv.second) + " 点");
        }
        std::sort(r.invalid_reasons.begin(), r.invalid_reasons.end());
    }

    if (r.valid_count == 0) {
        std::ostringstream oss;
        oss << (qPerPoint ? "未找到近件边缘与远件边缘的对应点"
                          : "沿轴向未找到近件边缘与远件边缘的对应点");
        reason = oss.str();
        return false;
    }
    return true;
}

// ============================================================
// §4.6 最大值、平台区与诊断
// ============================================================
// 单个量(间隙 |λ| 或 径向阶差 |Δe|)的最大值/平台区/并列候选
void MeasureWeldPreparation::findMaximumFor(Result& r, bool use_step, MaxItem& mx,
                                            double& median_out, bool& has_platform,
                                            double& pa0, double& pa1,
                                            double& ps0, double& ps1,
                                            std::vector<ExcludedCandidate>& excluded) const
{
    mx = MaxItem();
    has_platform = false;
    excluded.clear();
    const auto raw = [&](const Match& m) { return use_step ? m.step_abs : m.distance; };
    const auto sgn = [&](const Match& m) { return use_step ? m.step_signed : m.lambda; };

    std::vector<int> order;
    for (int k = 0; k < static_cast<int>(r.matches.size()); ++k) {
        if (r.matches[k].valid) order.push_back(k);
    }
    if (order.empty()) return;

    // 稳健峰值: 每点取其"边缘邻域"内有效点值的中值, 再在这个场里取最大。
    // 逐点取最大 = 取噪声上尾(N 点里最大 ~ 3.6σ), 对沿缝恒定的缺陷会系统性偏高约 0.3mm。
    std::vector<double> robust(order.size(), 0.0);
    for (size_t i = 0; i < order.size(); ++i) {
        const Match& mi = r.matches[order[i]];
        std::vector<double> nb;
        nb.reserve(order.size());
        for (size_t j = 0; j < order.size(); ++j) {
            const Match& mj = r.matches[order[j]];
            const double da = mj.near_a - mi.near_a;
            const double ds = frame_.arcDifference(mj.near_s, mi.near_s);
            if (std::sqrt(da * da + ds * ds) <= params_.edge_neighborhood) nb.push_back(raw(mj));
        }
        if (nb.empty()) { robust[i] = raw(mi); continue; }
        const size_t k = nb.size() / 2;
        std::nth_element(nb.begin(), nb.begin() + k, nb.end());
        robust[i] = nb[k];
    }
    std::vector<int> pos_of(r.matches.size(), -1);
    for (size_t i = 0; i < order.size(); ++i) pos_of[order[i]] = static_cast<int>(i);
    const auto val = [&](int matchIdx) { return robust[pos_of[matchIdx]]; };   // 稳健场(仅用于阶差支撑判据)

    // 整体中值: 全部有效点的中值。均匀缺陷下它就是该量的"水平值"(不受逐点取最大的噪声上尾影响)。
    {
        std::vector<double> all;
        all.reserve(order.size());
        for (int k : order) all.push_back(raw(r.matches[k]));
        const size_t m = all.size() / 2;
        std::nth_element(all.begin(), all.begin() + m, all.end());
        median_out = all[m];
    }

    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return raw(r.matches[a]) > raw(r.matches[b]);
    });

    // ---- 径向阶差的极值必须有邻域支撑 ----
    //   单个扫描坏点可以比邻域高好几个 mm, 但"最大阶差"说的是两个面的错位, 不是一个坏点。
    //   支撑判据: 以该点为中心、半径 = 边缘邻域内, 至少 3 个有效点的 |Δe| 不低于 (本点 - 容差)。
    //   不满足者进排除名单并写明理由(不静默丢弃); 若全部无支撑, 退回原始极值并置待复核。
    std::vector<int> cand;
    bool forceReview = false;
    if (!use_step) {
        cand = order;
    }
    else {
        const double supTol = std::max(2.0 * params_.grid_cell, 0.05);
        std::vector<ExcludedCandidate> unsupported;
        for (int k : order) {
            const double v = val(k);
            int sup = 0;
            for (int j : order) {
                const double da = r.matches[j].near_a - r.matches[k].near_a;
                const double ds = frame_.arcDifference(r.matches[j].near_s, r.matches[k].near_s);
                if (std::sqrt(da * da + ds * ds) > params_.edge_neighborhood) continue;
                if (val(j) < v - supTol) continue;
                ++sup;
            }
            if (sup >= 3) { cand.push_back(k); continue; }
            ExcludedCandidate ec;
            ec.distance = v;
            ec.near_index = r.matches[k].near_index;
            ec.reason = "邻域支撑点不足(" + std::to_string(sup)
                + " < 3, 半径 " + fmt(params_.edge_neighborhood, 1)
                + " mm 内无同等量级的点): 疑似孤立尖刺/坏点, 未计入最大阶差";
            unsupported.push_back(ec);
        }
        if (cand.empty()) { cand = order; forceReview = true; }
        excluded = unsupported;
        r.step_unsupported = static_cast<int>(unsupported.size());
    }
    const int best = cand.front();
    const Match& bm = r.matches[best];
    mx.ok = true;
    mx.value = raw(r.matches[best]);
    mx.lambda_signed = sgn(bm);
    mx.near_index = bm.near_index;
    mx.near_a = bm.near_a;
    mx.near_s = bm.near_s;
    mx.near_xyz = bm.near_xyz;
    mx.near_proj_xyz = bm.near_proj_xyz;
    mx.far_a = bm.far_a;
    mx.far_s = bm.far_s;
    mx.far_xyz = bm.far_xyz;

    // 并列/平台区
    const double tol = params_.platform_tol;
    double aLo = std::numeric_limits<double>::max(), aHi = -std::numeric_limits<double>::max();
    double sLo = std::numeric_limits<double>::max(), sHi = -std::numeric_limits<double>::max();
    for (int k : cand) {
        if (mx.value - raw(r.matches[k]) > tol) break;
        mx.ties.push_back(r.matches[k].near_index);
        aLo = std::min(aLo, r.matches[k].near_a); aHi = std::max(aHi, r.matches[k].near_a);
        sLo = std::min(sLo, r.matches[k].near_s); sHi = std::max(sHi, r.matches[k].near_s);
    }
    if (mx.ties.size() >= 2) {
        has_platform = true;
        pa0 = aLo; pa1 = aHi; ps0 = sLo; ps1 = sHi;
    }

    // 极值支持度
    {
        int nearCount = 0;
        for (int k : cand) {
            const double da = r.matches[k].near_a - bm.near_a;
            const double ds = frame_.arcDifference(r.matches[k].near_s, bm.near_s);
            if (std::sqrt(da * da + ds * ds) <= params_.edge_neighborhood) ++nearCount;
        }
        const double segSigma = (bm.far_seg >= 0
            && bm.far_seg < static_cast<int>(r.far_edge.refined_sigma.size()))
            ? r.far_edge.refined_sigma[bm.far_seg] : 0.0;
        if (nearCount < 3) {
            mx.needs_review = true;
            mx.review_reason = "极值邻域内有效点仅 " + std::to_string(nearCount) + " 个, 缺乏足够支持";
        }
        else if (segSigma > 3.0 * params_.grid_cell) {
            mx.needs_review = true;
            mx.review_reason = "极值处远件边缘局部拟合残差 " + fmt(segSigma, 3)
                + " mm 偏大(>3×栅格), 边缘定位不确定性高";
        }
        else {
            mx.review_reason = "极值邻域有效点 " + std::to_string(nearCount)
                + " 个; 远件边缘局部拟合残差 " + fmt(segSigma, 3) + " mm";
        }
        if (forceReview) {
            mx.needs_review = true;
            mx.review_reason += "; 全部候选均无邻域支撑, 该值仅作参考";
        }
        // 极值(含并列)若来自"扩窗重试"的配对, 必须提示人工确认:
        // 扩窗可能跨过缺失带连到更远的一段远边, 数值会偏大。
        bool fromWide = bm.widened;
        for (int k : cand) {
            if (mx.value - raw(r.matches[k]) > tol) break;
            if (r.matches[k].widened) { fromWide = true; break; }
        }
        if (fromWide) {
            mx.needs_review = true;
            mx.review_reason += "; 极值来自扩窗搜索(默认窗口内无对应边), 请人工确认该处配对";
        }
    }

    // 被排除的候选(附理由, 不静默隐藏)
    for (int k : cand) {
        if (k == best) continue;
        const Match& m = r.matches[k];
        if (raw(r.matches[k]) < mx.value - tol) break;
        ExcludedCandidate ec;
        ec.distance = raw(r.matches[k]);
        ec.near_index = m.near_index;
        ec.reason = "与最大值并列(差值 ≤ 平台容差 " + fmt(tol, 3)
            + " mm); 已作为并列候选列出";
        excluded.push_back(ec);
    }
    for (int k = 0; k < static_cast<int>(r.matches.size()); ++k) {
        if (r.matches[k].valid) continue;
        if (r.matches[k].invalid_reason.find("多交点") == std::string::npos) continue;
        ExcludedCandidate ec;
        ec.distance = std::numeric_limits<double>::quiet_NaN();
        ec.near_index = r.matches[k].near_index;
        ec.reason = "存在多交点歧义候选, 未计入极大值统计(位置 a="
            + fmt(r.matches[k].near_a, 1) + " mm, s=" + fmt(r.matches[k].near_s, 1) + " mm)";
        excluded.push_back(ec);
    }

    // 孤立丢点(长度不到一个点距量级)不构成"不可测区间", 不影响全局最大值结论;
    // 只有真正连成段的无交点区间才降级为部分有效(§4.6)。
    // 诊断列表限长(恒定值时全部点都是并列候选; 只保留前若干条并给出总数)
    if (excluded.size() > 20) {
        ExcludedCandidate ec;
        ec.distance = std::numeric_limits<double>::quiet_NaN();
        ec.near_index = -1;
        ec.reason = "其余 " + std::to_string(excluded.size() - 20)
            + " 条并列/存疑候选未逐条列出(总数 " + std::to_string(excluded.size()) + ")";
        excluded.resize(20);
        excluded.push_back(ec);
    }
}

void MeasureWeldPreparation::findMaximumAndDiagnostics(Result& r)
{
    // 间隙与径向阶差来自同一批配对点, 各自独立取最大/平台/并列候选
    findMaximumFor(r, /*use_step=*/false, r.maximum, r.gap_median, r.has_platform,
                   r.platform_a0, r.platform_a1, r.platform_s0, r.platform_s1, r.excluded);
    findMaximumFor(r, /*use_step=*/true, r.maximum_step, r.step_median, r.has_platform_step,
                   r.platform_step_a0, r.platform_step_a1,
                   r.platform_step_s0, r.platform_step_s1, r.excluded_step);
    r.is_global_max = r.missing_intervals.empty();
}

// ============================================================
// §7 报告
// ============================================================
void MeasureWeldPreparation::composeReport(Result& r) const
{
    // 只输出测量结果; 每条信息单独一行, 不挤在一起。
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    if (r.status == Status::Cancelled) {
        oss << "=== 焊前装配测量 ===\n已取消\n";
        r.report = oss.str();
        return;
    }
    const bool measurable = (r.status == Status::Valid || r.status == Status::Partial);
    if (!measurable) {
        oss << "=== 焊前装配测量结果 ===\n不可测";
        if (!r.reason.empty()) oss << ": " << r.reason;
        oss << "\n";
        r.report = oss.str();
        return;
    }

    oss << "=== 焊前装配测量结果（径向阶差: 正 = 近件在外）===\n";
    auto block = [&](const char* name, const MaxItem& mx, double median, bool hasPlat,
                     double pa0, double pa1, double ps0, double ps1) {
        oss << name << "整体中值: " << median << " mm\n";
        oss << name << "最大值: " << mx.value << " mm\n";
        oss << "  最大值位置（三维坐标）: " << fmt3(mx.near_xyz) << "\n";
        oss << "  最大值位置（展开坐标）: a=" << fmt(mx.near_a, 1)
            << " mm, s=" << fmt(mx.near_s, 1) << " mm\n";
        if (mx.needs_review) oss << "  最大值待复核: " << mx.review_reason << "\n";
        if (hasPlat) {
            oss << "  最大值位置不唯一（平台区）: a∈[" << fmt(pa0, 1) << ", " << fmt(pa1, 1)
                << "], s∈[" << fmt(ps0, 1) << ", " << fmt(ps1, 1) << "] mm\n";
        }
    };
    if (params_.metric != Metric::Gap && r.maximum_step.ok)
        block("径向阶差", r.maximum_step, r.step_median, r.has_platform_step,
              r.platform_step_a0, r.platform_step_a1, r.platform_step_s0, r.platform_step_s1);
    if (params_.metric != Metric::Step && r.maximum.ok)
        block("轴向间隙", r.maximum, r.gap_median, r.has_platform,
              r.platform_a0, r.platform_a1, r.platform_s0, r.platform_s1);

    oss << "覆盖:\n";
    oss << "  有效段: " << fmt(r.valid_length, 1) << " mm\n";
    oss << "  有效点: " << r.valid_count << "\n";
    oss << "  无效点: " << r.invalid_count << "\n";
    for (const auto& s : r.missing_intervals) oss << "  缺测: " << s << "\n";
    r.report = oss.str();
}

// ============================================================
// 可视化数据打包(只做数据准备; VTK/Qt 只能在 GUI 线程)
// ============================================================

// ============================================================
// 主流程
// ============================================================
MeasureWeldPreparation::Result MeasureWeldPreparation::evaluate()
{
    Result r;
    r.metric = params_.metric;
    r.gap_direction = params_.gap_direction;
    r.design_radius = params_.design_radius;
    r.single_cloud_roi = single_mode_;
    r.near_cloud_name = near_name_.empty() ? std::string("试样 A") : near_name_;
    r.far_cloud_name = far_name_.empty() ? std::string("试样 B") : far_name_;
    r.role_note = "未判定";

    std::string reason;
    if (!checkInputs(reason)) {
        r.status = Status::Failed;
        r.reason = reason;
        composeReport(r);
        return r;
    }

    if (!reportProgress(0, kStageTotal, "焊前测量: 校验输入...")) {
        r.status = Status::Cancelled; r.reason = "已取消"; composeReport(r); return r;
    }
    if (!resolveRoles(r, reason)) {
        r.status = Status::Unmeasurable;
        r.reason = reason;
        composeReport(r);
        return r;
    }

    if (!buildReferenceCylinder(r, reason)) {
        r.status = cancelled_ ? Status::Cancelled : Status::Unmeasurable;
        r.reason = reason;
        composeReport(r);
        return r;
    }
    if (!buildFrameAndProject(r, reason)) {
        r.status = cancelled_ ? Status::Cancelled : Status::Unmeasurable;
        r.reason = reason;
        composeReport(r);
        return r;
    }
    if (!buildEdges(r, reason)) {
        r.status = cancelled_ ? Status::Cancelled : Status::Unmeasurable;
        r.reason = reason;
        composeReport(r);
        return r;
    }
    if (!pairAlongDirection(r, reason)) {
        r.status = cancelled_ ? Status::Cancelled : Status::Unmeasurable;
        r.reason = reason;
        composeReport(r);
        return r;
    }

    if (!reportProgress(6, kStageTotal, "焊前测量: 汇总结果...")) {
        r.status = Status::Cancelled; r.reason = "已取消"; composeReport(r); return r;
    }
    findMaximumAndDiagnostics(r);

    r.status = r.missing_intervals.empty() ? Status::Valid : Status::Partial;
    if (r.status == Status::Partial) {
        r.reason.clear();   // 缺测区间已逐条列出, 不再附方法性说明
    }
    reportProgress(kStageTotal, kStageTotal, "焊前测量完成");
    composeReport(r);
    return r;
}
