#pragma once
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vtkPlaneSource.h>  

std::string to_string_with_precision(double num, int precision);
std::string GenerateRandomName(const std::string& prefix);