#pragma once
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vtkPlaneSource.h>  

std::string to_string_with_precision(double num, int precision);

//时间戳功能，监控性能
class TimeStamp {
   public:
       TimeStamp();
       ~TimeStamp();
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> end_time;
    void Start();
    void End();
    std::string get_QString_time_duration();
};

vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients& coefficients, double x, double y, double z, float scale[2] = nullptr);

