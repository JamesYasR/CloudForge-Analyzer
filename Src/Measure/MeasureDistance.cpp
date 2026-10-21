#include "Measure/MeasureDistance.h"
#include <cmath>

MeasureDistance::Result MeasureDistance::compute(const Point& start, const Point& end)
{
    Result result;
    result.start = start;
    result.end = end;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(start[i]) || !std::isfinite(end[i])) return result;
    }
    result.distance = std::hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
    result.success = std::isfinite(result.distance);
    return result;
}
