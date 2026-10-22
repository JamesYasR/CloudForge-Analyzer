#pragma once
#include "Measure/MeasureHeight.h"
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

struct HeightCase {
    std::string name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr measured, reference;
};
inline HeightCase heightCase(int index)
{
    HeightCase item{std::to_string(index), pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(), pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    for (int x = -12; x <= 12; ++x) for (int y = -9; y <= 9; ++y) {
        const float plane = index == 2 ? .125f * x - .25f * y + 3 : 0;
        item.reference->emplace_back(x, y, plane);
        float height = index == 0 ? 2.0f : index == 1 ? 2 + .0625f * x : index == 2 ? 1.5f + .03125f * y : index == 3 ? 0 : -2 + .0625f * x;
        item.measured->emplace_back(x, y, plane + height);
    }
    return item;
}
inline std::string heightSnapshot(MeasureHeight& measurement, const std::vector<double>& averages,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colors)
{
    std::ostringstream out;
    out << std::hexfloat << measurement.GetMaxDistance() << ' ' << measurement.GetMinDistance() << ' ' << measurement.GetMeanDistance();
    for (auto value : measurement.GetPlaneCoefficients()->values) out << ' ' << value;
    for (auto value : averages) out << ' ' << value;
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& p : *colors) for (auto channel : {p.r, p.g, p.b}) { hash ^= channel; hash *= 1099511628211ull; }
    out << ' ' << hash << '\n';
    return out.str();
}
inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalHeightColors(const HeightCase& item, MeasureHeight& measurement)
{
    auto colors = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto coeff = measurement.GetPlaneCoefficients();
    std::vector<double> heights;
    double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
    const double a = coeff->values[0], b = coeff->values[1], c = coeff->values[2], d = coeff->values[3];
    const double denominator = std::sqrt(a*a+b*b+c*c);
    for (const auto& p : *item.measured) {
        const double value = std::abs(a*p.x+b*p.y+c*p.z+d)/denominator;
        heights.push_back(value); lo = std::min(lo,value); hi = std::max(hi,value);
    }
    double range = hi-lo; if (range == 0) range = 1;
    for (size_t i = 0; i < item.measured->size(); ++i) {
        const auto& source = (*item.measured)[i];
        pcl::PointXYZRGB p; p.x=source.x; p.y=source.y; p.z=source.z;
        const double normalized = (heights[i]-lo)/range;
        if (normalized < .5) { p.r=static_cast<std::uint8_t>(255*(normalized*2)); p.g=255; p.b=0; }
        else { p.r=255; p.g=static_cast<std::uint8_t>(255*(2-normalized*2)); p.b=0; }
        colors->push_back(p);
    }
    return colors;
}
