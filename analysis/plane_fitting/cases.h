#pragma once
#include "Fitting/Fit_Plane.h"
#include <cmath>
#include <random>

struct PlaneCase {
    std::string name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Fit_Plane::FitParams params;
    float oldLocalRadius=5, oldAnomalyThreshold=1;
};
inline PlaneCase planeCase(int number, int side = 32)
{
    PlaneCase result;
    result.name = "case_" + std::to_string(number);
    result.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::mt19937 random(1729);
    std::uniform_real_distribution<float> noise(-.04f,.04f), scattered(-5,5);
    const float spacing = number == 6 ? .25f : .5f;
    for (int y=0;y<side;++y) for (int x=0;x<side;++x) {
        float px=x*spacing, py=y*spacing, pz=noise(random);
        if (number == 0) pz=0;
        if (number == 1) pz += .12f*px-.24f*py+3;
        if (number == 2 && (y*side+x)%31==0) pz += 3;
        if (number == 3 && (y*side+x)%31==0) pz += 10;
        if (number == 4 && random()%5==0) pz += scattered(random);
        if (number == 5) pz += .04f*(px-side*spacing/2)*(px-side*spacing/2);
        if (number == 7 && y>side/2) pz += (y-side/2)*.6f;
        if (number == 8 && (y*side+x)%7==0) pz += .99f;
        if (number == 9 && (y*side+x)%7==0) pz += 1.01f;
        if (number == 10) { px += 10000; py -= 20000; pz += 30000; }
        if (number == 11) { float temp=px; px=pz; pz=temp; }
        if (number == 12 && (y*side+x)%7==0) pz += -1.01f;
        if (number == 13) pz += .4f*std::sin(px)*std::cos(py);
        if (number == 14) pz += .3f*scattered(random);
        result.cloud->emplace_back(px,py,pz);
    }
    if (number == 15) result.params.PlaneFitThreshold=.05f;
    if (number == 16) result.params.PlaneFitThreshold=.25f;
    if (number == 17) result.params.MaxIterations=50;
    return result;
}
