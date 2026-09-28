#include "Basic/CylinderSurfaceFrame.h"

#include <cmath>
#include <sstream>

namespace {

// 判断 double 是否有限
inline bool finiteD(double v) { return std::isfinite(v); }

inline bool finiteV(const Eigen::Vector3d& v)
{
    return finiteD(v.x()) && finiteD(v.y()) && finiteD(v.z());
}

// 构造与 u 垂直的确定性正交基: 取 u 分量绝对值最小的坐标轴做 Gram-Schmidt。
// 用"最小分量轴"是为了数值稳定; 用固定规则(而不是随机)是为了让同一 u
// 每次都得到同一组 v/w, 保证近件与远件、以及多次运行之间的展开一致。
void buildBasis(const Eigen::Vector3d& u, Eigen::Vector3d& v, Eigen::Vector3d& w)
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
    if (nw > 1e-12) {
        w /= nw;
        // 重新正交化 v, 保证 v ⊥ w ⊥ u 精确成立
        v = w.cross(u).normalized();
    }
}

} // namespace

double CylinderSurfaceFrame::wrapToPi(double dphi)
{
    if (!finiteD(dphi)) return 0.0;
    double r = std::fmod(dphi + M_PI, 2.0 * M_PI);
    if (r < 0.0) r += 2.0 * M_PI;
    return r - M_PI;
}

bool CylinderSurfaceFrame::init(const Eigen::Vector3d& axisPoint,
                                const Eigen::Vector3d& axisDirection,
                                double designRadius,
                                double phi0)
{
    valid_ = false;
    if (!finiteV(axisPoint) || !finiteV(axisDirection)) return false;
    if (!finiteD(designRadius) || !finiteD(phi0)) return false;
    if (!(designRadius > 0.0)) return false;
    const double n = axisDirection.norm();
    if (!(n > 1e-9)) return false;

    O_ = axisPoint;
    u_ = axisDirection / n;
    R_ = designRadius;
    phi0_ = wrapToPi(phi0);
    buildBasis(u_, v_, w_);
    valid_ = true;
    return true;
}

bool CylinderSurfaceFrame::project(const Eigen::Vector3d& P,
                                   double& a, double& s, double& e) const
{
    if (!valid_ || !finiteV(P)) return false;
    const Eigen::Vector3d d = P - O_;
    a = d.dot(u_);
    const Eigen::Vector3d r = d - a * u_;
    const double rho = r.norm();
    e = rho - R_;
    const double phi = std::atan2(r.dot(w_), r.dot(v_));
    s = R_ * wrapToPi(phi - phi0_);
    return true;
}

Eigen::Vector3d CylinderSurfaceFrame::unproject(double a, double s, double radialOffset) const
{
    if (!valid_) return Eigen::Vector3d::Zero();
    const double phi = phi0_ + s / R_;
    const double rho = R_ + radialOffset;
    return O_ + a * u_ + rho * (std::cos(phi) * v_ + std::sin(phi) * w_);
}

double CylinderSurfaceFrame::arcDifference(double s1, double s2) const
{
    if (!valid_) return 0.0;
    return R_ * wrapToPi((s1 - s2) / R_);
}

bool CylinderSurfaceFrame::rawPhi(const Eigen::Vector3d& P, double& phi) const
{
    if (!valid_ || !finiteV(P)) return false;
    const Eigen::Vector3d d = P - O_;
    const Eigen::Vector3d r = d - d.dot(u_) * u_;
    phi = std::atan2(r.dot(w_), r.dot(v_));
    return true;
}

double CylinderSurfaceFrame::choosePhi0(const std::vector<double>& phis)
{
    if (phis.empty()) return 0.0;
    double sx = 0.0, sy = 0.0;
    int n = 0;
    for (double p : phis) {
        if (!finiteD(p)) continue;
        sx += std::cos(p);
        sy += std::sin(p);
        ++n;
    }
    if (n == 0) return 0.0;
    // s = R·wrapToPi(phi - phi0) 的不连续点在 phi = phi0 ± π。
    // 取 phi0 = 数据圆均值, 切口便落在数据反面(phi_c + π), 被测区域角度连续。
    // (取"圆均值 + π"会把切口正好放进数据中间, 那是错的。)
    return std::atan2(sy, sx);
}

std::string CylinderSurfaceFrame::describe() const
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    if (!valid_) {
        oss << "参考圆柱坐标系: 未建立";
        return oss.str();
    }
    oss << "参考圆柱口径: 投影到设计柱面(R=" << R_ << " mm)上的轴向/周向尺寸; "
        << "轴向正方向 u=(" << u_.x() << ", " << u_.y() << ", " << u_.z() << "); "
        << "角度零点 phi0=" << phi0_ << " rad; 展开分支 s∈(-piR, piR]";
    return oss.str();
}
