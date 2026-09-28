// ============================================================
// test_weld_prep.cpp —— 焊前装配阶差/间隙 无界面验证程序(不进 CMakeLists)
//
// 依据: docs/当前需新增功能/焊前装配阶差与间隙-独立执行方案.md §5
//
// 仿真从**解析几何真值**生成点云, 不从测量算法的输出反推答案:
//     P(a,s) = O + a·u + [R + h(a,s)]·[cos(s/R) v + sin(s/R) w]
// 两件的边缘分别由 w = 0(近边)与 w = d(u)(远边)解析定义, 其中
//   w = (x-C)·n,  u = (x-C)·t,  t 为接缝切向, n ⊥ t。
// 于是真值有闭式解:
//   阶差(同轴向截面 a=常数)      = d / |cos ψ|
//   间隙·沿圆柱轴向              = d / |sin ψ|
//   间隙·曲面内接缝法向          = d
// (ψ 为 t 与 +a 轴夹角。)所以 Ψ=0 ⇒ 纵向直缝: 阶差可测、轴向间隙退化不可测;
// Ψ=90° ⇒ 环向直缝: 轴向/法向间隙一致、阶差退化不可测。
//
// 分层(§5.2): A=几何核心(真值轴线) / B=边缘测量(真值轴线) / C=完整计算(远件拟合)。
// 用法: ./build/test_weld_prep [--all] [--quick] [--out DIR]
// ============================================================

#include "Basic/CylinderSurfaceFrame.h"
#include "Measure/MeasureWeldPreparation.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr double kPi = 3.14159265358979323846;
static const double kR = 1940.0;          // 设计半径(mm) —— 代表场景, 不是软件默认真值

// ------------------------------------------------------------
// 仿真规格
// ------------------------------------------------------------
struct Bump { double u0, amp, sigma; };

struct Spec {
    std::string name;
    double radius = kR;             // 设计半径 R(mm); 方案 §5.1 要求同时设置其他 R
    double psi_deg = 0.0;           // 接缝切向与 +a 轴夹角
    double d0 = 2.0;                // 基础边缘间距(沿 n 拟合, mm)
    std::vector<Bump> bumps;        // d(u) = d0 + Σ amp·exp(-((u-u0)/sigma)^2)
    double near_radial_offset = 0.0;// 近件整体径向偏置(用于验证阶差不混入径向高低差)
    double form_amp = 0.0;          // 毫米级平滑形面变化幅度
    double noise_sigma = 0.0;       // 表面噪声(mm)
    double spike_h = 0.0;           // 近边缘单点尖刺高度(mm)
    double pitch = 0.5;             // 采样点距(mm)
    double a_half = 150.0;          // 轴向半宽(mm)
    double s_half = 100.0;          // 周向半宽(mm)
    double piece_width = 60.0;      // 每件沿 n 的宽度(mm)
    bool   drop_far_band = false;   // 远边缺失带(遮挡/裁剪)
    double drop_u0 = -20.0, drop_u1 = 20.0;
    bool   slot_near = false;       // 把近件切开一条槽(制造同一条缝上的真实断点)
    double slot_u0 = -5.0, slot_u1 = 5.0;
    double second_seam = 0.0;       // >0 时在 w=second_seam 处再加一条平行接缝(真正多缝)
    bool   rigid_transform = false; // 施加刚体位姿
    int    seed = 12345;
};

struct Sample {
    pcl::PointCloud<pcl::PointXYZ>::Ptr nearCloud{ new pcl::PointCloud<pcl::PointXYZ> };
    pcl::PointCloud<pcl::PointXYZ>::Ptr farCloud{ new pcl::PointCloud<pcl::PointXYZ> };
    Eigen::Vector3d O = Eigen::Vector3d::Zero();
    Eigen::Vector3d u = Eigen::Vector3d::UnitZ();
    double R = kR;
    double zNear = 0.0, zFar = 0.0;
    // 连续边缘(解析)真值
    double truthStepRadial = 0.0; // 径向阶差真值 = 近件整体径向偏置(近件在外为正)
    double truthStep = 0.0;       // [旧口径参考] d/|cosψ|(周向弧长投影, 已不作为交付口径)
    double truthGapAxial = 0.0;   // d/|sinψ|
    double truthGapNormal = 0.0;  // d
    // 有限采样点对应的真值(方案 §5.1: 两类真值必须分开)
    double sampStep = 0.0, sampGapAxial = 0.0, sampGapNormal = 0.0;
    long   sampCount = 0;
    int nearPoints = 0, farPoints = 0;
};

static std::string f2(double v, int p = 3)
{
    if (!std::isfinite(v)) return "N/A";
    std::ostringstream o;
    o.setf(std::ios::fixed); o.precision(p); o << v;
    return o.str();
}

// ------------------------------------------------------------
// 从解析真值生成一对试样点云
// ------------------------------------------------------------
static Sample buildSample(const Spec& sp)
{
    Sample s;
    s.R = sp.radius;

    // 让被测区域落在"周向位移会产生明显扫描 Z 差"的位置:
    // u 与 Z 轴倾斜 60°, 且数据放在 phi≈0 一带 => dZ/ds ≈ 0.87 mm/mm,
    // 两件的 Z 均值差可达数十毫米, 便于按 §3 的约定命名近远件。
    const double tilt = 60.0 * kPi / 180.0;
    Eigen::Vector3d uaxis(std::sin(tilt), 0.0, std::cos(tilt));
    CylinderSurfaceFrame F;
    F.init(Eigen::Vector3d::Zero(), uaxis, sp.radius, 0.0);
    s.O = F.axisPoint();
    s.u = F.axisDirection();

    const double psi = sp.psi_deg * kPi / 180.0;
    const Eigen::Vector2d t(std::cos(psi), std::sin(psi));
    Eigen::Vector2d n(-std::sin(psi), std::cos(psi));

    // 近远命名约定(§3): "Z 增大代表远离扫描仪" ⇒ Z 均值较小者为近件。
    // 近件放在 n 的负侧, g 取区域中心处 (dZ/da, dZ/ds); 需要 n·g > 0。
    // 用真值坐标系数值估计 g。
    {
        const double eps = 0.5;
        const Eigen::Vector3d p0 = F.unproject(0.0, 0.0, 0.0);
        const Eigen::Vector3d pa = F.unproject(eps, 0.0, 0.0);
        const Eigen::Vector3d ps = F.unproject(0.0, eps, 0.0);
        const Eigen::Vector2d g((pa.z() - p0.z()) / eps, (ps.z() - p0.z()) / eps);
        if (n.dot(g) < 0.0) n = -n;      // 翻转后保证近件在 -n 侧时 Z 更小
    }

    std::mt19937 rng(static_cast<unsigned>(sp.seed));
    std::normal_distribution<double> nd(0.0, 1.0);

    auto dOf = [&](double uu) {
        double d = sp.d0;
        for (const auto& b : sp.bumps) {
            const double z = (uu - b.u0) / b.sigma;
            d += b.amp * std::exp(-z * z);
        }
        return d;
    };

    std::vector<int> nearU;   // 记录近边点索引(用于尖刺注入)
    pcl::PointCloud<pcl::PointXYZ> nearC, farC;

    const int na = static_cast<int>(std::lround(2 * sp.a_half / sp.pitch)) + 1;
    const int ns = static_cast<int>(std::lround(2 * sp.s_half / sp.pitch)) + 1;
    for (int ia = 0; ia < na; ++ia) {
        const double a = -sp.a_half + ia * sp.pitch;
        for (int is = 0; is < ns; ++is) {
            const double ss = -sp.s_half + is * sp.pitch;
            const double uu = a * t.x() + ss * t.y();
            const double w = a * n.x() + ss * n.y();
            const double d = dOf(uu);
            // eps 必须留: 否则 w 与 0/d 相差 1e-16 时会把整行采样点判到错误的一侧,
            // 制造出"边缘位置随 s 跳半格"的假象(那不是算法误差)。
            const double kEps = 1e-9;

            bool isNear = (w >= -sp.piece_width - kEps && w <= kEps);
            bool isFar = (w >= d - kEps && w <= sp.piece_width + kEps);
            if (sp.second_seam > 0.0) {
                const double off = sp.second_seam;
                isNear = isNear || (w >= off - 15.0 && w <= off);
                isFar = isFar || (w >= off + d && w <= off + 15.0);
            }
            if (!isNear && !isFar) continue;

            if (sp.slot_near && isNear && uu >= sp.slot_u0 && uu <= sp.slot_u1) continue;
            if (sp.drop_far_band && isFar && uu >= sp.drop_u0 && uu <= sp.drop_u1) continue;

            double h = 0.0;
            if (isNear) h += sp.near_radial_offset;
            if (sp.form_amp != 0.0) {
                h += sp.form_amp * std::sin(2.0 * kPi * a / 220.0) *
                     std::cos(2.0 * kPi * ss / 260.0);
            }
            if (sp.noise_sigma > 0.0) h += sp.noise_sigma * nd(rng);

            const Eigen::Vector3d P = F.unproject(a, ss, h);
            if (isNear) {
                nearU.push_back(static_cast<int>(nearC.size()));
                nearC.push_back(pcl::PointXYZ(static_cast<float>(P.x()),
                                              static_cast<float>(P.y()),
                                              static_cast<float>(P.z())));
            }
            else {
                farC.push_back(pcl::PointXYZ(static_cast<float>(P.x()),
                                             static_cast<float>(P.y()),
                                             static_cast<float>(P.z())));
            }
        }
    }

    // 近边缘单点尖刺(§5.3 "单点尖刺")
    if (sp.spike_h > 0.0 && !nearU.empty()) {
        // 取最靠近 (a≈60, s≈0) 的近边点(边缘上), 沿径向抬高
        double best = 1e30;
        int bestIdx = -1;
        for (int idx : nearU) {
            const auto& p = nearC.points[idx];
            Eigen::Vector3d P(p.x, p.y, p.z);
            double aa = 0, sv = 0, ev = 0;
            F.project(P, aa, sv, ev);
            if (std::fabs(sv) > 0.6) continue;         // 只挑边缘行
            const double cost = std::fabs(aa - 60.0);
            if (cost < best) { best = cost; bestIdx = idx; }
        }
        if (bestIdx >= 0) {
            const auto& p = nearC.points[bestIdx];
            Eigen::Vector3d P(p.x, p.y, p.z);
            double aa = 0, sv = 0, ev = 0;
            F.project(P, aa, sv, ev);
            const Eigen::Vector3d Q = F.unproject(aa, sv, ev + sp.spike_h);
            nearC.points[bestIdx] = pcl::PointXYZ(static_cast<float>(Q.x()),
                                                  static_cast<float>(Q.y()),
                                                  static_cast<float>(Q.z()));
        }
    }

    // 刚体位姿(§5.3 旋转不变性)
    if (sp.rigid_transform) {
        Eigen::Matrix3d Rm;
        const double ax = 0.62, ay = -0.41, az = 0.77, th = 0.93;
        Rm = Eigen::AngleAxisd(th, Eigen::Vector3d(ax, ay, az).normalized());
        const Eigen::Vector3d tr(1234.5, -678.9, 321.0);
        auto apply = [&](pcl::PointCloud<pcl::PointXYZ>& c) {
            for (auto& p : c.points) {
                const Eigen::Vector3d P(p.x, p.y, p.z);
                const Eigen::Vector3d Q = Rm * P + tr;
                p = pcl::PointXYZ(static_cast<float>(Q.x()),
                                  static_cast<float>(Q.y()),
                                  static_cast<float>(Q.z()));
            }
        };
        apply(nearC);
        apply(farC);
        s.O = Rm * s.O + tr;
        s.u = (Rm * s.u).normalized();
    }

    nearC.width = static_cast<std::uint32_t>(nearC.size()); nearC.height = 1; nearC.is_dense = true;
    farC.width = static_cast<std::uint32_t>(farC.size());   farC.height = 1;  farC.is_dense = true;

    // ---- 真值: 阶差/间隙的闭式解 ----
    const double ac = std::fabs(std::cos(psi));
    const double as = std::fabs(std::sin(psi));
    s.truthGapNormal = sp.d0;
    for (const auto& b : sp.bumps) s.truthGapNormal = std::max(s.truthGapNormal, sp.d0 + b.amp);
    s.truthStep = (ac > 1e-9) ? s.truthGapNormal / ac : std::numeric_limits<double>::infinity();
    s.truthGapAxial = (as > 1e-9) ? s.truthGapNormal / as : std::numeric_limits<double>::infinity();
    s.truthStepRadial = sp.near_radial_offset;

    // ---- 有限采样点真值: 只用"近件接缝侧边缘采样点"和解析远边线 ----
    // 近件边缘采样点 = 该格点在 +n 方向上的相邻格不在近件内。
    {
        const int daN = (n.x() > 0) ? 1 : -1;
        const int dsN = (n.y() > 0) ? 1 : -1;
        const bool hasBumps = !sp.bumps.empty();
        for (int ia = 0; ia < na; ++ia) {
            const double a = -sp.a_half + ia * sp.pitch;
            for (int is = 0; is < ns; ++is) {
                const double ss = -sp.s_half + is * sp.pitch;
                const double w = a * n.x() + ss * n.y();
                const double uu = a * t.x() + ss * t.y();
                if (!(w >= -sp.piece_width - 1e-9 && w <= 1e-9)) continue;
                if (sp.slot_near && uu >= sp.slot_u0 && uu <= sp.slot_u1) continue;
                // 沿 +n(即 +w)方向走一个采样格的相邻点是否仍属于近件。
                // 只有"最外侧那一排"才是接缝侧边缘采样点 —— 这正是测量侧
                // 邻域角度空缺判据要挑出来的集合。
                const int sa = (n.x() > 1e-9) ? 1 : ((n.x() < -1e-9) ? -1 : 0);
                const int sb = (n.y() > 1e-9) ? 1 : ((n.y() < -1e-9) ? -1 : 0);
                const int ja = ia + sa;
                const int js = is + sb;
                const bool inRange = (ja >= 0 && ja < na && js >= 0 && js < ns);
                bool nbIn = false;
                if (inRange) {
                    const double a2 = -sp.a_half + ja * sp.pitch;
                    const double s2 = -sp.s_half + js * sp.pitch;
                    const double w2 = a2 * n.x() + s2 * n.y();
                    const double u2 = a2 * t.x() + s2 * t.y();
                    nbIn = (w2 >= -sp.piece_width - 1e-9 && w2 <= 1e-9);
                    if (sp.slot_near && u2 >= sp.slot_u0 && u2 <= sp.slot_u1) nbIn = false;
                }
                if (nbIn) continue;                       // 内部点
                if (!inRange && w < -1e-6) continue;      // 越界的矩形边界, 不是接缝边
                ++s.sampCount;

                // 远边也是**采样**出来的: 必须把解析交点量化到实际采样网格上
                // (沿 +n 方向的第一个采样位置), 否则近边量化、远边解析, 会凭空多出
                // 最多一个点距的偏差, 把采样量化差误当成算法误差。
                auto quantS = [&](double val) {
                    const double k = std::ceil((val + sp.s_half) / sp.pitch - 1e-9);
                    return -sp.s_half + k * sp.pitch;
                };
                auto quantA = [&](double val) {
                    const double k = std::ceil((val + sp.a_half) / sp.pitch - 1e-9);
                    return -sp.a_half + k * sp.pitch;
                };

                // 阶差: 同一 a 上求远边(w = d)交点
                if (!hasBumps && std::fabs(n.y()) > 1e-6) {
                    const double sQ = quantS((sp.d0 - n.x() * a) / n.y());
                    s.sampStep = std::max(s.sampStep, std::fabs(ss - sQ));
                }
                else if (hasBumps && std::fabs(n.x()) < 1e-6) {
                    const double sQ = quantS(dOf(a));   // ψ=0 时远边就是 s = d(a)
                    s.sampStep = std::max(s.sampStep, std::fabs(ss - sQ));
                }
                // 间隙·沿轴向: 同一 s 上求远边交点
                if (!hasBumps && std::fabs(n.x()) > 1e-6) {
                    const double aQ = quantA((sp.d0 - n.y() * ss) / n.x());
                    s.sampGapAxial = std::max(s.sampGapAxial, std::fabs(a - aQ));
                }
                // 间隙·曲面内接缝法向: 到远边的垂距
                if (!hasBumps) {
                    s.sampGapNormal = std::max(s.sampGapNormal,
                        std::fabs(sp.d0 - (n.x() * a + n.y() * ss)));
                }
                else if (std::fabs(n.x()) < 1e-6) {
                    s.sampGapNormal = std::max(s.sampGapNormal,
                        std::fabs(ss - dOf(a)));
                }
            }
        }
        (void)daN; (void)dsN;
    }

    // ---- 按 Z 均值命名近远件 ----
    double zn = 0.0, zf = 0.0;
    for (const auto& p : nearC.points) zn += p.z;
    for (const auto& p : farC.points) zf += p.z;
    zn /= std::max<size_t>(1, nearC.size());
    zf /= std::max<size_t>(1, farC.size());
    s.nearPoints = static_cast<int>(nearC.size());
    s.farPoints = static_cast<int>(farC.size());
    if (zn <= zf) {
        *s.nearCloud = nearC; *s.farCloud = farC; s.zNear = zn; s.zFar = zf;
    }
    else {
        *s.nearCloud = farC; *s.farCloud = nearC; s.zNear = zf; s.zFar = zn;
    }
    return s;
}

// ------------------------------------------------------------
// 一次验证
// ------------------------------------------------------------
struct Check {
    std::string case_name;
    std::string layer;         // A / B / C
    bool   pass = false;
    std::string detail;
};

static bool approx(double v, double truth, double tol) { return std::fabs(v - truth) <= tol; }

// 运行一次并打印关键行
static void runCase(const Spec& sp, const std::string& layer, bool fix_axis,
                    const fs::path& outDir, std::vector<Check>& checks,
                    const std::string& expect, int perf_points_hint)
{
    Sample s = buildSample(sp);

    MeasureWeldPreparation mp;
    mp.setNearCloud(s.nearCloud, "sample_A");
    mp.setFarCloud(s.farCloud, "sample_B");
    MeasureWeldPreparation::Params wp;
    wp.design_radius = s.R;
    wp.metric = MeasureWeldPreparation::Metric::Both;   // 一次配对同时给出径向阶差与间隙
    wp.gap_direction = MeasureWeldPreparation::GapDirection::Axial;
    if (fix_axis) mp.setFixedReferenceAxis(s.O, s.u);
    mp.setParams(wp);

    const auto t0 = std::chrono::steady_clock::now();
    MeasureWeldPreparation::Result stepRes = mp.evaluate();
    const double tStep = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    // 间隙(沿轴向)
    MeasureWeldPreparation mp2;
    mp2.setNearCloud(s.nearCloud, "sample_A");
    mp2.setFarCloud(s.farCloud, "sample_B");
    MeasureWeldPreparation::Params wp2 = wp;
    wp2.metric = MeasureWeldPreparation::Metric::Gap;
    wp2.gap_direction = MeasureWeldPreparation::GapDirection::Axial;
    if (fix_axis) mp2.setFixedReferenceAxis(s.O, s.u);
    mp2.setParams(wp2);
    const auto t1 = std::chrono::steady_clock::now();
    MeasureWeldPreparation::Result gapAxRes = mp2.evaluate();
    const double tGap = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t1).count();

    // 间隙(曲面内接缝法向)
    MeasureWeldPreparation mp3;
    mp3.setNearCloud(s.nearCloud, "sample_A");
    mp3.setFarCloud(s.farCloud, "sample_B");
    MeasureWeldPreparation::Params wp3 = wp;
    wp3.metric = MeasureWeldPreparation::Metric::Gap;
    wp3.gap_direction = MeasureWeldPreparation::GapDirection::SeamNormal;
    if (fix_axis) mp3.setFixedReferenceAxis(s.O, s.u);
    mp3.setParams(wp3);
    MeasureWeldPreparation::Result gapNmRes = mp3.evaluate();

    auto statusName = [](MeasureWeldPreparation::Status st) -> const char* {
        switch (st) {
        case MeasureWeldPreparation::Status::Valid: return "Valid";
        case MeasureWeldPreparation::Status::Partial: return "Partial";
        case MeasureWeldPreparation::Status::Unmeasurable: return "Unmeasurable";
        case MeasureWeldPreparation::Status::Cancelled: return "Cancelled";
        default: return "Failed";
        }
    };
    auto maxOf = [](const MeasureWeldPreparation::Result& r) {
        return r.maximum.ok ? r.maximum.value : std::numeric_limits<double>::quiet_NaN();
    };
    auto maxStepOf = [](const MeasureWeldPreparation::Result& r) {
        return r.maximum_step.ok ? r.maximum_step.value
                                 : std::numeric_limits<double>::quiet_NaN();
    };

    std::cout << "[CASE] " << sp.name << " layer=" << layer
              << " psi=" << f2(sp.psi_deg, 1) << " d0=" << f2(sp.d0, 3)
              << " near=" << s.nearPoints << " far=" << s.farPoints
              << " zNear=" << f2(s.zNear, 2) << " zFar=" << f2(s.zFar, 2) << "\n";
    std::cout << "[KEY] " << sp.name << "/step_radial status=" << statusName(stepRes.status)
              << " max=" << f2(maxStepOf(stepRes)) << " truth=" << f2(s.truthStepRadial)
              << " err=" << f2(maxStepOf(stepRes) - s.truthStepRadial)
              << " valid=" << stepRes.valid_count << " invalid=" << stepRes.invalid_count
              << " len=" << f2(stepRes.valid_length, 1)
              << " t=" << f2(tStep, 2) << "s\n";
    std::cout << "[KEY] " << sp.name << "/gap_axial status=" << statusName(gapAxRes.status)
              << " max=" << f2(maxOf(gapAxRes)) << " truth=" << f2(s.truthGapAxial)
              << " err=" << f2(maxOf(gapAxRes) - s.truthGapAxial)
              << " valid=" << gapAxRes.valid_count << " invalid=" << gapAxRes.invalid_count
              << " t=" << f2(tGap, 2) << "s\n";
    std::cout << "[KEY] " << sp.name << "/gap_normal status=" << statusName(gapNmRes.status)
              << " max=" << f2(maxOf(gapNmRes)) << " truth=" << f2(s.truthGapNormal)
              << " err=" << f2(maxOf(gapNmRes) - s.truthGapNormal)
              << " valid=" << gapNmRes.valid_count << " invalid=" << gapNmRes.invalid_count << "\n";
    std::cout << "[KEY] " << sp.name << "/step_radial_normal status=" << statusName(gapNmRes.status)
              << " max=" << f2(maxStepOf(gapNmRes)) << " truth=" << f2(s.truthStepRadial)
              << " err=" << f2(maxStepOf(gapNmRes) - s.truthStepRadial) << "\n";
    // 前 3 个最大对应关系(便于核对 0.x mm 级偏差来自哪一侧)
    {
        std::vector<const MeasureWeldPreparation::Match*> vm;
        for (const auto& m : stepRes.matches) if (m.valid) vm.push_back(&m);
        std::sort(vm.begin(), vm.end(), [](auto a, auto b) { return a->step_abs > b->step_abs; });
        for (size_t i = 0; i < vm.size() && i < 3; ++i) {
            std::cout << "[TOP] " << sp.name << "/step d=" << f2(vm[i]->step_abs)
                      << " near(a=" << f2(vm[i]->near_a, 2) << ",s=" << f2(vm[i]->near_s, 2)
                      << ") far(a=" << f2(vm[i]->far_a, 2) << ",s=" << f2(vm[i]->far_s, 2)
                      << ")\n";
        }
        vm.clear();
        for (const auto& m : gapAxRes.matches) if (m.valid) vm.push_back(&m);
        std::sort(vm.begin(), vm.end(), [](auto a, auto b) { return a->distance > b->distance; });
        for (size_t i = 0; i < vm.size() && i < 3; ++i) {
            std::cout << "[TOP] " << sp.name << "/gap d=" << f2(vm[i]->distance)
                      << " near(a=" << f2(vm[i]->near_a, 2) << ",s=" << f2(vm[i]->near_s, 2)
                      << ") far(a=" << f2(vm[i]->far_a, 2) << ",s=" << f2(vm[i]->far_s, 2)
                      << ")\n";
        }
    }
    if (!stepRes.baseline_note.empty())
        std::cout << "[BASE] " << sp.name << " " << stepRes.baseline_note << "\n";
    if (!stepRes.reason.empty())
        std::cout << "[WHY] " << sp.name << "/step " << stepRes.reason << "\n";
    if (!gapAxRes.reason.empty())
        std::cout << "[WHY] " << sp.name << "/gap_axial " << gapAxRes.reason << "\n";
    // 边缘候选被拒原因的分布(便于定位"边缘无观测支持"这类失败)
    {
        auto dumpReasons = [&](const char* tag, const MeasureWeldPreparation::Edge& e) {
            std::map<std::string, int> cnt;
            for (const auto& p : e.points) {
                if (!p.valid) cnt[p.reject_reason]++;
            }
            std::vector<std::pair<int, std::string>> v;
            for (const auto& kv : cnt) v.push_back({ kv.second, kv.first });
            std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
            for (size_t i = 0; i < v.size() && i < 6; ++i) {
                std::cout << "[EDGE] " << sp.name << "/" << tag << " reject x"
                          << v[i].first << ": " << v[i].second << "\n";
            }
            std::cout << "[EDGE] " << sp.name << "/" << tag << " " << e.note << "\n";
            if (sp.name == "oblique60" || sp.name == "circ_gap3") {
                std::cout << "[EDGE] " << sp.name << "/" << tag << " aRange=["
                          << f2(e.a_min, 2) << "," << f2(e.a_max, 2) << "] sRange=["
                          << f2(e.s_min, 2) << "," << f2(e.s_max, 2) << "]\n";
                int shown = 0;
                for (const auto& p : e.points) {
                    if (!p.valid) continue;
                    if (p.a < -44.0 || p.a > -43.0) continue;
                    std::cout << "       valid a=" << f2(p.a, 2) << " s=" << f2(p.s, 2)
                              << " support=" << f2(p.support, 2) << " outward="
                              << f2(p.outward_deg, 0) << " toOther=" << f2(p.to_other_mm, 1) << "\n";
                    if (++shown >= 6) break;
                }
            }
        };
        dumpReasons("near", stepRes.near_edge);
        dumpReasons("far", stepRes.far_edge);
    }
    for (const auto& mi : stepRes.missing_intervals)
        std::cout << "[MISS] " << sp.name << "/step " << mi << "\n";
    for (const auto& mi : gapAxRes.missing_intervals)
        std::cout << "[MISS] " << sp.name << "/gap_axial " << mi << "\n";
    std::cout << "[TRUTH] " << sp.name << " samples=" << s.sampCount
              << " stepRadial=" << f2(s.truthStepRadial)
              << " | step: sampled=" << f2(s.sampStep) << " continuous=" << f2(s.truthStep)
              << " | gapAxial: sampled=" << f2(s.sampGapAxial)
              << " continuous=" << f2(s.truthGapAxial)
              << " | gapNormal: sampled=" << f2(s.sampGapNormal)
              << " continuous=" << f2(s.truthGapNormal) << "\n";
    std::cout << "[EXPECT] " << sp.name << " " << expect << "\n";

    // ---- 报告样例(供人工过目: 精简后的客户可见输出) ----
    if (sp.name == "circ_gap3" || sp.name == "long_step2" || sp.name == "radial_offset") {
        std::cout << "---- REPORT " << sp.name << " / 方向=轴向 ----\n"
                  << stepRes.report << "---- END ----\n";
        if (sp.name == "radial_offset") {
            std::cout << "---- REPORT " << sp.name << " / 方向=接缝法向 ----\n"
                      << gapNmRes.report << "---- END ----\n";
        }
    }

    // ---- 保存样本与真值(可复现) ----
    if (!outDir.empty() && sp.name.rfind("large_", 0) != 0) {
        const fs::path dir = outDir / sp.name;
        fs::create_directories(dir);
        pcl::io::savePCDFileBinary((dir / "near.pcd").string(), *s.nearCloud);
        pcl::io::savePCDFileBinary((dir / "far.pcd").string(), *s.farCloud);
        std::ofstream tf(dir / "truth.txt");
        tf.setf(std::ios::fixed); tf.precision(6);
        tf << "case " << sp.name << "\n"
           << "layer " << layer << "\n"
           << "R " << s.R << "\n"
           << "axis_point " << s.O.x() << " " << s.O.y() << " " << s.O.z() << "\n"
           << "axis_dir " << s.u.x() << " " << s.u.y() << " " << s.u.z() << "\n"
           << "psi_deg " << sp.psi_deg << "\n"
           << "d0 " << sp.d0 << "\n"
           << "bumps " << sp.bumps.size() << "\n";
        for (const auto& b : sp.bumps)
            tf << "bump " << b.u0 << " " << b.amp << " " << b.sigma << "\n";
        tf << "truth_step_radial " << s.truthStepRadial << "\n"
           << "truth_step_circ_ref " << s.truthStep << "\n"
           << "truth_gap_axial " << s.truthGapAxial << "\n"
           << "truth_gap_normal " << s.truthGapNormal << "\n"
           << "z_near " << s.zNear << "\n"
           << "z_far " << s.zFar << "\n"
           << "near_points " << s.nearPoints << "\n"
           << "far_points " << s.farPoints << "\n"
           << "near_radial_offset " << sp.near_radial_offset << "\n"
           << "form_amp " << sp.form_amp << "\n"
           << "noise_sigma " << sp.noise_sigma << "\n"
           << "pitch " << sp.pitch << "\n"
           << "seed " << sp.seed << "\n"
           << "rigid_transform " << (sp.rigid_transform ? 1 : 0) << "\n";
    }

    // ---- 断言 ----
    auto add = [&](const std::string& nm, bool ok, const std::string& det) {
        Check c; c.case_name = sp.name + "/" + nm; c.layer = layer;
        c.pass = ok; c.detail = det; checks.push_back(c);
    };

    // ---- PCD 往返复测: GUI 路径必然经过 float32 PCD, 这里确认量化后结论不变 ----
    if (!outDir.empty() && sp.name.rfind("large_", 0) != 0) {
        const fs::path dir = outDir / sp.name;
        pcl::PointCloud<pcl::PointXYZ>::Ptr nPcd(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fPcd(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile((dir / "near.pcd").string(), *nPcd) == 0 &&
            pcl::io::loadPCDFile((dir / "far.pcd").string(), *fPcd) == 0) {
            MeasureWeldPreparation mp4;
            mp4.setNearCloud(nPcd, "near");
            mp4.setFarCloud(fPcd, "far");
            MeasureWeldPreparation::Params wp4 = wp;
            if (fix_axis) mp4.setFixedReferenceAxis(s.O, s.u);
            mp4.setParams(wp4);
            const auto r4 = mp4.evaluate();
            std::cout << "[PCD] " << sp.name << " stepStatus="
                      << statusName(r4.status) << " stepMax="
                      << f2(r4.maximum.ok ? r4.maximum.value
                                          : std::numeric_limits<double>::quiet_NaN())
                      << " (内存结果 " << f2(maxOf(stepRes)) << ")\n";
            add("pcd_roundtrip_matches",
                (r4.status == stepRes.status) &&
                (!r4.maximum.ok || !stepRes.maximum.ok ||
                 std::fabs(r4.maximum.value - stepRes.maximum.value) < 1e-6),
                "pcd=" + f2(r4.maximum.ok ? r4.maximum.value : 0.0)
                + " mem=" + f2(stepRes.maximum.ok ? stepRes.maximum.value : 0.0));
        }
    }


    const bool expect_reject = (sp.name == "dual_seam");
    // ---- 验收判据: 用"几何推导的允许区间", 而不是一个拍出来的容差 ----
    // 连续边缘真值 -> 采样真值 之间存在点距量化; 垂直于量测方向的量化误差会被
    // 1/|cosψ| 或 1/|sinψ| 放大到量测方向上。算法还会对远边阶梯做局部直线平滑,
    // 因此再给一份同样的平滑预算。可接受区间 =
    //     [ min(连续, 采样) , max(连续, 采样) + 0.5·点距 / 方向放大因子 ]
    const double psiRad = sp.psi_deg * kPi / 180.0;
    const double ampStep = (std::fabs(std::cos(psiRad)) > 1e-6)
        ? 0.5 * sp.pitch / std::fabs(std::cos(psiRad)) : std::numeric_limits<double>::infinity();
    const double ampAxial = (std::fabs(std::sin(psiRad)) > 1e-6)
        ? 0.5 * sp.pitch / std::fabs(std::sin(psiRad)) : std::numeric_limits<double>::infinity();
    auto inBand = [](double v, double cont, double samp, double amp) {
        if (!std::isfinite(amp)) return false;
        const double lo = std::min(cont, samp) - 1e-6;
        const double hi = std::max(cont, samp) + amp + 1e-6;
        return v >= lo && v <= hi;
    };
    auto bandText = [](double cont, double samp, double amp) {
        std::ostringstream o; o.setf(std::ios::fixed); o.precision(3);
        o << "[" << std::min(cont, samp) << ", " << std::max(cont, samp) + amp << "]";
        return o.str();
    };
    // 主判据: 与"有限采样点真值"比(方案 §5.1); 连续真值只作参考输出。
    // ---- 径向阶差(口径 2026-09: 阶差 = 径向 e_P - e_Q, 近件在外为正) ----
    //   与间隙来自同一次配对: 配对成立 -> 两个量都有; 配对不成立 -> 两个量都不给(不得单独编数)。
    //   容差 = 边缘点噪声(两侧各一次, 3σ) + 跨缝几毫米内形面变化 + 量化/平滑余量。
    const double tolStep = 0.20 + 3.0 * sp.noise_sigma + 0.15 * sp.form_amp;
    auto checkStep = [&](const MeasureWeldPreparation::Result& r, const char* tag) {
        const bool measurable = (r.status == MeasureWeldPreparation::Status::Valid ||
                                 r.status == MeasureWeldPreparation::Status::Partial);
        if (!measurable) {
            add(std::string(tag) + "_absent_when_unpaired",
                !r.maximum_step.ok,
                std::string("status=") + statusName(r.status)
                + " step_ok=" + (r.maximum_step.ok ? "1" : "0"));
            return;
        }
        const double got = maxStepOf(r);
        const double err = std::fabs(got - s.truthStepRadial);
        add(std::string(tag) + "_value",
            r.maximum_step.ok && err <= tolStep,
            "step=" + f2(got) + " truth=" + f2(s.truthStepRadial)
            + " |err|=" + f2(err) + " tol=" + f2(tolStep));
    };
    if (expect_reject) {
        add("step_rejected_as_multi_seam",
            stepRes.status == MeasureWeldPreparation::Status::Unmeasurable
            && !stepRes.maximum_step.ok,
            std::string("status=") + statusName(stepRes.status));
    }
    else {
        checkStep(stepRes, "step_radial");           // 轴向配对(环缝)
        checkStep(gapNmRes, "step_radial_normal");   // 接缝法向配对(独立几何交叉验证)
    }
    if (expect_reject) {
        // 多缝案例: 三个指标都不得给出数值结论
    }
    else if (std::isfinite(s.truthGapAxial) && s.sampGapAxial > 0.0) {
        add("gap_axial_value",
            (gapAxRes.status == MeasureWeldPreparation::Status::Valid ||
             gapAxRes.status == MeasureWeldPreparation::Status::Partial) &&
            inBand(maxOf(gapAxRes), s.truthGapAxial, s.sampGapAxial, ampAxial),
            "max=" + f2(maxOf(gapAxRes)) + " 允许区间="
            + bandText(s.truthGapAxial, s.sampGapAxial, ampAxial)
            + " (连续 " + f2(s.truthGapAxial) + " / 采样 " + f2(s.sampGapAxial) + ")");
    }
    else {
        add("gap_axial_unmeasurable",
            gapAxRes.status == MeasureWeldPreparation::Status::Unmeasurable,
            std::string("status=") + statusName(gapAxRes.status));
    }
    if (sp.name == "missing_band") {
        add("partial_with_missing_interval",
            gapAxRes.status == MeasureWeldPreparation::Status::Partial &&
            !gapAxRes.missing_intervals.empty(),
            std::string("gap_axial status=") + statusName(gapAxRes.status) +
            " missing=" + std::to_string(gapAxRes.missing_intervals.size()));
    }
    if (sp.name == "broken_near") {
        // psi=0(纵向缝): 可测的配对是"接缝法向"模式, 断点应表现为缺测区间
        add("partial_with_missing_interval",
            gapNmRes.status == MeasureWeldPreparation::Status::Partial &&
            !gapNmRes.missing_intervals.empty(),
            std::string("gap_normal status=") + statusName(gapNmRes.status) +
            " missing=" + std::to_string(gapNmRes.missing_intervals.size()));
    }
    if (sp.name == "dual_seam") {
        add("multi_seam_rejected",
            stepRes.status == MeasureWeldPreparation::Status::Unmeasurable,
            std::string("step status=") + statusName(stepRes.status));
    }
    if (sp.name == "spike") {
        // 尖刺不得被静默吞掉: 要么被排除并留下诊断, 要么体现在极值复核标记上
        add("spike_traceable",
            gapNmRes.maximum.needs_review || !gapNmRes.excluded.empty() ||
            gapNmRes.invalid_count > 0,
            "needs_review=" + std::string(gapNmRes.maximum.needs_review ? "1" : "0") +
            " excluded=" + std::to_string(gapNmRes.excluded.size()) +
            " invalid=" + std::to_string(gapNmRes.invalid_count));
        // 径向阶差: 单个坏点不得成为"最大阶差"(必须落到排除名单, 且极值回到邻域量级)
        add("spike_not_max_step",
            !gapNmRes.excluded_step.empty() && gapNmRes.maximum_step.value < 1.0,
            "excluded_step=" + std::to_string(gapNmRes.excluded_step.size()) +
            " max_step=" + f2(maxStepOf(gapNmRes)));
    }
    if (sp.name == "dual_peak") {
        add("platform_reported",
            gapNmRes.has_platform && gapNmRes.maximum.ties.size() >= 2,
            "has_platform=" + std::string(gapNmRes.has_platform ? "1" : "0") +
            " ties=" + std::to_string(gapNmRes.maximum.ties.size()));
    }
    if (sp.name == "local_peak") {
        add("peak_preserved",
            gapNmRes.maximum.ok && approx(gapNmRes.maximum.value, 4.0, 0.5),
            "max=" + f2(gapNmRes.maximum.value));
    }
    (void)perf_points_hint;
}

// ------------------------------------------------------------
int main(int argc, char** argv)
{
    fs::path outDir = "analysis/weld_preparation/samples";
    bool quick = false, big = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--quick") quick = true;
        else if (a == "--big") big = true;
    }
    fs::create_directories(outDir);

    std::vector<Check> checks;
    std::cout << "[INFO] R=" << kR << " mm; 真值由解析几何给出, 测量程序只读点云。\n";

    // ---- A/B 层: 真值轴线(快) ----
    {
        Spec sp; sp.name = "long_step2"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        runCase(sp, "A/B", true, outDir, checks,
                "纵向直缝: 阶差=2mm 可测; 沿轴向间隙因平行退化不可测", 0);
    }
    {
        Spec sp; sp.name = "circ_gap3"; sp.psi_deg = 90.0; sp.d0 = 3.0;
        runCase(sp, "A/B", true, outDir, checks,
                "环向直缝: 轴向/法向间隙一致=3mm; 阶差不强求可测", 0);
    }
    {
        Spec sp; sp.name = "oblique60"; sp.psi_deg = 60.0; sp.d0 = 1.5;
        runCase(sp, "A/B", true, outDir, checks,
                "斜缝60°: 阶差=3.0, 轴向间隙=1.732, 法向间隙=1.5, 三者互不相同", 0);
    }
    {
        Spec sp; sp.name = "local_peak"; sp.psi_deg = 0.0; sp.d0 = 1.0;
        sp.bumps.push_back({ 0.0, 3.0, 30.0 });
        runCase(sp, "A/B", true, outDir, checks,
                "局部窄张口: 峰值 4.0mm 必须保留, 不被全局/局部拟合压平", 0);
    }
    {
        Spec sp; sp.name = "dual_peak"; sp.psi_deg = 0.0; sp.d0 = 1.0;
        sp.bumps.push_back({ -40.0, 3.0, 15.0 });
        sp.bumps.push_back({ 40.0, 3.0, 15.0 });
        runCase(sp, "A/B", true, outDir, checks,
                "两个并列极大值: 返回平台区/并列候选而不是伪造唯一位置", 0);
    }
    {
        Spec sp; sp.name = "radial_offset"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.near_radial_offset = 1.0;
        runCase(sp, "A/B", true, outDir, checks,
                "近件径向抬高 1mm: 阶差仍=2mm(不混入径向高低差)", 0);
    }
    {
        Spec sp; sp.name = "form_noise"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.form_amp = 0.8; sp.noise_sigma = 0.05;
        runCase(sp, "A/B", true, outDir, checks,
                "mm 级平滑形面变化 + 噪声: 阶差仍=2mm(口径不引入形面偏差)", 0);
    }
    {
        Spec sp; sp.name = "rigid_xform"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.rigid_transform = true;
        runCase(sp, "A/B", true, outDir, checks,
                "整体旋转平移: 尺寸不变、位置随变换一致", 0);
    }
    {
        Spec sp; sp.name = "missing_band"; sp.psi_deg = 90.0; sp.d0 = 3.0;
        sp.drop_far_band = true; sp.drop_u0 = -25.0; sp.drop_u1 = 25.0;
        runCase(sp, "A/B", true, outDir, checks,
                "远边局部缺失: 报 Partial + 缺测区间; 缺口不得当成 0 间隙", 0);
    }
    {
        Spec sp; sp.name = "spike"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.spike_h = 4.0;
        runCase(sp, "A/B", true, outDir, checks,
                "近边缘单点尖刺: 极值诊断可追溯, 不被静默吞掉", 0);
    }
    {
        Spec sp; sp.name = "dual_seam"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.second_seam = 40.0;
        runCase(sp, "A/B", true, outDir, checks,
                "区域内两条平行但不同位置的接缝: 第一版拒绝并给出可执行原因", 0);
    }

    {
        // 强曲率: R=300 mm + 240 mm 弧段(45.8°), 弓高 23.8 mm —— 与 R=1940 的浅弧形成对照,
        // 用来确认几何核心不是"只在近似平面上成立"。
        // piece_width 同时决定每件沿 n 的宽度; psi=0 时它=周向弧长跨度,
        // 所以必须一起放大才能得到真正的强曲率(240mm 弧 / R=300 => 45.8°, 弓高 23.8mm)。
        Spec sp; sp.name = "smallR_curved"; sp.radius = 300.0;
        sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.pitch = 0.8; sp.a_half = 100.0; sp.s_half = 240.0; sp.piece_width = 240.0;
        runCase(sp, "A/B", true, outDir, checks,
                "强曲率 R=300mm/45.8°弧: 阶差与浅弧案例同样精确", 0);
    }
    {
        Spec sp; sp.name = "broken_near"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.slot_near = true; sp.slot_u0 = -6.0; sp.slot_u1 = 6.0;
        runCase(sp, "A/B", true, outDir, checks,
                "近件在同一接缝上被切开 12mm: 视为断点, 报缺测区间而不是拒绝", 0);
    }

    // ---- 规模案例(§5.3 "小/中/200-350 万点"): 用 --big 打开 ----
    if (big) {
        Spec sp; sp.name = "large_500k"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        sp.pitch = 0.35; sp.a_half = 150.0; sp.s_half = 100.0;
        runCase(sp, "A/B", true, outDir, checks,
                "约 50 万点: 结果一致性与分阶段耗时", 0);
        Spec sp2; sp2.name = "large_3m"; sp2.psi_deg = 0.0; sp2.d0 = 2.0;
        sp2.pitch = 0.10; sp2.a_half = 200.0; sp2.s_half = 120.0;
        runCase(sp2, "A/B", true, outDir, checks,
                "约 320 万点: 结果一致性与峰值内存", 0);
    }

    // ---- C 层: 远件自拟合(慢, 只跑代表案例) ----
    if (!quick) {
        Spec sp; sp.name = "long_step2_fit"; sp.psi_deg = 0.0; sp.d0 = 2.0;
        runCase(sp, "C", false, outDir, checks,
                "完整链路(远件固定半径自拟合): 阶差=2mm, 基准稳定性须通过", 0);
        Spec sp3; sp3.name = "noisy_fit"; sp3.psi_deg = 0.0; sp3.d0 = 2.0;
        sp3.noise_sigma = 0.05;
        runCase(sp3, "C", false, outDir, checks,
                "有噪声(0.05mm)的远件自拟合: 检验浅弧下的基准稳定性闸门", 0);
        Spec sp2; sp2.name = "circ_gap3_fit"; sp2.psi_deg = 90.0; sp2.d0 = 3.0;
        runCase(sp2, "C", false, outDir, checks,
                "完整链路(环向): 轴向间隙=3mm, 基准稳定性须通过", 0);
    }

    // ---- 汇总 ----
    int pass = 0, fail = 0;
    for (const auto& c : checks) {
        if (c.pass) ++pass; else ++fail;
        std::cout << (c.pass ? "[PASS] " : "[FAIL] ") << c.case_name
                  << " (" << c.layer << ") " << c.detail << "\n";
    }
    std::cout << "[SUMMARY] pass=" << pass << " fail=" << fail
              << " total=" << checks.size() << "\n";
    return fail == 0 ? 0 : 1;
}
