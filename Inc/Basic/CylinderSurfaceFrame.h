#pragma once
// ============================================================
// CylinderSurfaceFrame —— 参考圆柱柱面坐标系(纯数学, 不依赖 Qt / VTK / 查看器)
//
// 依据: docs/当前需新增功能/焊前装配阶差与间隙-独立执行方案.md §2.1
//
// 把三维点投影到"设计参考圆柱"上, 得到三个量(计算全程 double, 并先减去轴上一点 O):
//     a = (P-O)·u                     轴向坐标(mm)
//     r = (P-O) - a·u
//     phi = atan2(r·w, r·v)
//     s = R · unwrap(phi - phi0)      周向弧长(mm)
//     e = |r| - R                     到理想柱面的带符号径向偏差(mm)
// 回投: O + a·u + R·(cos(phi) v + sin(phi) w),  phi = phi0 + s/R
//
// 重要约定(方案 §2.1):
//   * 近件与远件必须使用同一个 O、u、R、phi0 与角度展开分支; 禁止分别给两件
//     设零角度后再相减, 否则两件的周向坐标不可比。
//   * phi0 应选在测量区域之外(推荐 choosePhi0(): 把分支切口放到数据反面),
//     这样接缝区域的角度展开连续, ±π 处的跳变不会落在被测区域里。
//   * 展开的"unwrap"是相对 phi0 定义的单一分支: s ∈ (-πR, πR]。因此被测区域
//     的周向跨度必须小于半周; 跨度接近 πR 时展开本身已不可信, 调用方应拒绝测量
//     (见 MeasureWeldPreparation 的跨度检查)。
//   * 这里测的是"投影到设计参考圆柱上的尺寸"。理想圆柱可以无拉伸展开, 但不代表
//     有形面偏差的真实板面投影后仍严格保长; 该口径与 R 的来源必须进入报告。
// ============================================================

#include <Eigen/Dense>
#include <string>
#include <vector>

class CylinderSurfaceFrame
{
public:
    CylinderSurfaceFrame() = default;

    // 建立坐标系。返回 false 表示参数非法(方向为零 / 半径非正 / 含 NaN)。
    // axisPoint 取轴上任意一点即可(投影只用到 O 到轴的距离, 沿轴位置不影响 a/s/e)。
    bool init(const Eigen::Vector3d& axisPoint,
              const Eigen::Vector3d& axisDirection,
              double designRadius,
              double phi0 = 0.0);

    bool valid() const { return valid_; }
    double radius() const { return R_; }
    double phi0() const { return phi0_; }
    const Eigen::Vector3d& axisPoint() const { return O_; }
    const Eigen::Vector3d& axisDirection() const { return u_; }
    const Eigen::Vector3d& refV() const { return v_; }
    const Eigen::Vector3d& refW() const { return w_; }

    // 三维点 -> (a, s, e)。未初始化时返回 false。
    bool project(const Eigen::Vector3d& P, double& a, double& s, double& e) const;

    // (a, s) + 可选径向偏置 -> 三维点(默认贴在设计柱面上)。
    Eigen::Vector3d unproject(double a, double s, double radialOffset = 0.0) const;

    // 周向弧长差 s1 - s2, 已经过 ±πR 缠绕(同一分支下才有意义)。
    double arcDifference(double s1, double s2) const;

    // 弧长 <-> 圆心角
    double arcToAngle(double ds) const { return R_ > 0.0 ? ds / R_ : 0.0; }
    double angleToArc(double dphi) const { return dphi * R_; }

    // 把角度缠绕到 (-π, π]
    static double wrapToPi(double dphi);

    // 由一组角度(弧度, 未缠绕亦可)推荐一个 phi0: 取圆均值后转 π,
    // 即把 ±π 分支切口放到数据分布的反面。数据为空时返回 0。
    static double choosePhi0(const std::vector<double>& phis);

    // 直接给出某个三维点的原始角度(未减去 phi0), 供 choosePhi0 使用。
    bool rawPhi(const Eigen::Vector3d& P, double& phi) const;

    // 诊断文本(仅描述口径, 供报告使用)
    std::string describe() const;

private:
    bool   valid_ = false;
    Eigen::Vector3d O_ = Eigen::Vector3d::Zero();   // 轴上一点
    Eigen::Vector3d u_ = Eigen::Vector3d::UnitZ();  // 单位轴向
    Eigen::Vector3d v_ = Eigen::Vector3d::UnitX();  // 参考径向(phi=phi0 处)
    Eigen::Vector3d w_ = Eigen::Vector3d::UnitY();  // w = u × v
    double R_ = 0.0;        // 设计半径(mm)
    double phi0_ = 0.0;     // 角度零点(弧度)
};
