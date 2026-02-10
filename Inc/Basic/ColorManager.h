#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
class ColorManager {
public:
    ColorManager(double R, double G, double B);
    ColorManager();
    ~ColorManager();
    double r;
    double g;
    double b;
};