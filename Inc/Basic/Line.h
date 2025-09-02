#pragma once
#include "Basic/ColorManager.h"
#include "config/pcl114.h"

class Line{
public:
    Line(pcl::PointXYZ startp, pcl::PointXYZ endp, ColorManager colm, double wid, Eigen::VectorXf coef);
    Line();
    ~Line()=default;
    pcl::PointXYZ start;
    pcl::PointXYZ end;
    ColorManager color;
    double width;
    Eigen::VectorXf coeffs;
    Eigen::Vector3f dir_vector;
};
