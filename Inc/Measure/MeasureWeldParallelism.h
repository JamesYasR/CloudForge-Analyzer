#pragma once
#include "Basic/CylinderSurfaceFrame.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <functional>
#include <string>
#include <utility>

class MeasureWeldParallelism
{
public:
    struct Cylinder {
        Eigen::Vector3d point=Eigen::Vector3d::Zero();
        Eigen::Vector3d axis=Eigen::Vector3d::UnitZ();
        double radius=0;
    };
    struct Result {
        bool success=false;
        bool cancelled=false;
        std::string message;
        double angleDegrees=0;
        double comparisonLength=0;
        CylinderSurfaceFrame frame;
        std::array<std::vector<Eigen::Vector2d>,2> projected,centers;
        std::array<Eigen::Vector2d,2> linePoint,lineDirection;
        std::array<double,2> centerlineRms{},radialOffset{};
        std::array<std::vector<Eigen::Vector3d>,2> curves;
    };
    using Cloud=pcl::PointCloud<pcl::PointXYZ>;
    using ProgressCallback=std::function<bool(int,int,const std::string&)>;
    MeasureWeldParallelism(Cloud::ConstPtr first,Cloud::ConstPtr second,Cylinder cylinder);
    void setProgressCallback(ProgressCallback callback) { progress_=std::move(callback); }
    Result compute() const;
private:
    std::array<Cloud::ConstPtr,2> clouds_;
    Cylinder cylinder_;
    ProgressCallback progress_;
};
