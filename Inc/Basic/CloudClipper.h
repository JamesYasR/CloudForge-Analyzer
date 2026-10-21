#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <functional>
#include <string>
#include <vector>

class CloudClipper
{
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZ>;
    using Point2D = std::array<double, 2>;
    struct Projection {
        std::array<double, 16> matrix{};
        std::array<double, 4> viewport{0, 0, 1, 1};
        bool operator==(const Projection& other) const { return matrix == other.matrix && viewport == other.viewport; }
    };
    struct Request {
        Cloud::ConstPtr cloud;
        std::vector<Point2D> polygon;
        Projection projection;
    };
    struct Result {
        Cloud::Ptr inside{new Cloud};
        Cloud::Ptr outside{new Cloud};
        bool success = false;
        bool cancelled = false;
        std::string error;
    };
    using ProgressCallback = std::function<bool(int, int, const std::string&)>;

    static std::string validatePolygon(const std::vector<Point2D>& polygon);
    static bool contains(const std::vector<Point2D>& polygon, const Point2D& point);
    static Result compute(const Request& request, const ProgressCallback& progress = {});
};
