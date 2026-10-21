#include "Basic/CloudClipper.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
using Point = CloudClipper::Point2D;
constexpr double epsilon = 1e-10;

double cross(const Point& a, const Point& b, const Point& p)
{
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
}

bool onSegment(const Point& a, const Point& b, const Point& p)
{
    return std::abs(cross(a, b, p)) <= epsilon
        && p[0] >= std::min(a[0], b[0]) - epsilon && p[0] <= std::max(a[0], b[0]) + epsilon
        && p[1] >= std::min(a[1], b[1]) - epsilon && p[1] <= std::max(a[1], b[1]) + epsilon;
}

bool intersects(const Point& a, const Point& b, const Point& c, const Point& d)
{
    const double abC = cross(a, b, c), abD = cross(a, b, d);
    const double cdA = cross(c, d, a), cdB = cross(c, d, b);
    return (((abC > epsilon && abD < -epsilon) || (abC < -epsilon && abD > epsilon))
        && ((cdA > epsilon && cdB < -epsilon) || (cdA < -epsilon && cdB > epsilon)))
        || onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
}
}

std::string CloudClipper::validatePolygon(const std::vector<Point2D>& polygon)
{
    if (polygon.size() < 3) return "请至少选取三个顶点";
    double area = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        if (!std::isfinite(a[0]) || !std::isfinite(a[1])) return "请重新绘制轮廓";
        if (std::hypot(b[0] - a[0], b[1] - a[1]) < 1e-8) return "相邻顶点重合，请调整轮廓";
        area += a[0] * b[1] - b[0] * a[1];
        for (size_t j = i + 1; j < polygon.size(); ++j) {
            if (j == i + 1 || (i == 0 && j == polygon.size() - 1)) continue;
            if (intersects(a, b, polygon[j], polygon[(j + 1) % polygon.size()])) return "轮廓交叉，请调整顶点";
        }
    }
    if (std::abs(area) < 1e-9) return "轮廓没有围成区域，请调整顶点";
    return {};
}

bool CloudClipper::contains(const std::vector<Point2D>& polygon, const Point2D& point)
{
    bool inside = false;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        if (onSegment(a, b, point)) return true;
        if ((a[1] > point[1]) != (b[1] > point[1])
            && point[0] < a[0] + (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1])) inside = !inside;
    }
    return inside;
}

CloudClipper::Result CloudClipper::compute(const Request& request, const ProgressCallback& progress)
{
    Result result;
    result.error = validatePolygon(request.polygon);
    if (!result.error.empty()) return result;
    if (!request.cloud || request.cloud->empty()) { result.error = "没有可裁切的点云"; return result; }
    for (double value : request.projection.matrix) {
        if (!std::isfinite(value)) { result.error = "当前视角无法裁切，请重新调整视角"; return result; }
    }
    const auto& viewport = request.projection.viewport;
    if (!std::all_of(viewport.begin(), viewport.end(), [](double value) { return std::isfinite(value); })
        || viewport[0] >= viewport[2] || viewport[1] >= viewport[3]) {
        result.error = "当前视窗无法裁切，请重新调整视窗";
        return result;
    }
    auto prepare = [&](Cloud& output) {
        output.header = request.cloud->header;
        output.sensor_origin_ = request.cloud->sensor_origin_;
        output.sensor_orientation_ = request.cloud->sensor_orientation_;
        output.is_dense = true;
        output.reserve(request.cloud->size());
    };
    prepare(*result.inside);
    prepare(*result.outside);
    Point2D lo{1, 1}, hi{0, 0};
    for (const auto& point : request.polygon) for (int k = 0; k < 2; ++k) {
        lo[k] = std::min(lo[k], point[k]);
        hi[k] = std::max(hi[k], point[k]);
    }
    const auto& m = request.projection.matrix;
    const auto& v = request.projection.viewport;
    const size_t total = request.cloud->size();
    for (size_t i = 0; i < total; ++i) {
        if (i % 8192 == 0 && progress && !progress(int(i), int(total), "预览裁切")) {
            result.cancelled = true;
            result.inside->clear();
            result.outside->clear();
            return result;
        }
        const auto& p = (*request.cloud)[i];
        const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        bool inside = false;
        if (finite) {
            const double w = m[12] * p.x + m[13] * p.y + m[14] * p.z + m[15];
            if (w > 1e-12) {
                const double x = (m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3]) / w;
                const double y = (m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7]) / w;
                const double z = (m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]) / w;
                const Point2D screen{v[0] + (x + 1) * 0.5 * (v[2] - v[0]),
                                     1 - v[1] - (y + 1) * 0.5 * (v[3] - v[1])};
                inside = x >= -1 - epsilon && x <= 1 + epsilon && y >= -1 - epsilon && y <= 1 + epsilon
                    && z >= -epsilon && z <= 1 + epsilon
                    && screen[0] >= lo[0] - epsilon && screen[0] <= hi[0] + epsilon
                    && screen[1] >= lo[1] - epsilon && screen[1] <= hi[1] + epsilon
                    && contains(request.polygon, screen);
            }
        }
        auto& output = inside ? result.inside : result.outside;
        output->points.push_back(p);
        if (!finite) output->is_dense = false;
    }
    for (auto output : {result.inside, result.outside}) {
        output->width = static_cast<std::uint32_t>(output->size());
        output->height = 1;
    }
    if (progress && !progress(int(total), int(total), "预览完成")) {
        result.cancelled = true;
        return result;
    }
    result.success = true;
    return result;
}
