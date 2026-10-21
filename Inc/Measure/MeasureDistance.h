#pragma once

#include <array>

class MeasureDistance
{
public:
    using Point = std::array<double, 3>;
    struct Result {
        Point start{};
        Point end{};
        double distance = 0.0;
        bool success = false;
    };

    static Result compute(const Point& start, const Point& end);
};
