#include "Fitting/Fit_Plane.h"
#include "BaselineFitPlane.h"
#include "cases.h"
#include <array>
#include <set>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
static std::array<unsigned,3> key(const pcl::PointXYZ& p)
{
    std::array<unsigned,3> result;
    std::memcpy(result.data(),&p.x,3*sizeof(float));
    return result;
}
static int differences()
{
    bool valid=true;
    for (int i=0;i<18;++i) {
        auto item=planeCase(i);
        BaselineFitPlane::FitParams p;
        p.LocalRadius=item.oldLocalRadius;p.AnomalyThreshold=item.oldAnomalyThreshold;
        p.PlaneFitThreshold=item.params.PlaneFitThreshold;p.MaxIterations=item.params.MaxIterations;
        BaselineFitPlane old(item.cloud,p);old.compute();
        Fit_Plane current(item.cloud,item.params);current.compute();
        std::set<std::array<unsigned,3>> original;
        for (const auto& point:*old.Get_Inliers()) original.insert(key(point));
        size_t shared=0;for (const auto& point:*current.Get_Inliers()) shared+=original.count(key(point));
        const auto a=old.Get_Coeff_in().cast<double>().eval();
        auto b=current.Get_Coeff_in().cast<double>().eval();
        const double dot=a.head<3>().dot(b.head<3>());
        const double angle=std::acos(std::clamp(std::abs(dot)/(a.head<3>().norm()*b.head<3>().norm()),0.,1.))*180/std::acos(-1.);
        if (dot<0) b=-b;
        double maximum=0;
        for (const auto& point:*item.cloud) maximum=std::max(maximum,std::abs((a[0]-b[0])*point.x+(a[1]-b[1])*point.y+(a[2]-b[2])*point.z+a[3]-b[3]));
        if (i==2 || i==3) valid &= current.Get_Outliers()->size()==(item.cloud->size()+30)/31;
        std::cout<<std::setprecision(9)<<"[DELTA] case="<<i<<" old_anomaly="<<old.Get_AnomalyPoints()->size()
            <<" old_inliers="<<old.Get_Inliers()->size()<<" old_outliers="<<old.Get_Outliers()->size()
            <<" new_inliers="<<current.Get_Inliers()->size()<<" new_outliers="<<current.Get_Outliers()->size()
            <<" inlier_changes="<<old.Get_Inliers()->size()+current.Get_Inliers()->size()-2*shared
            <<" normal_angle_deg="<<angle<<" max_signed_distance_change_mm="<<maximum
            <<" old_rmse="<<old.ComputeRMSE()<<" new_rmse="<<current.ComputeRMSE()<<std::endl;
    }
    std::cout<<"known_spikes_rejected="<<valid<<std::endl;
    return valid ? 0:1;
}
int main(int argc,char** argv)
{
    if (argc>1 && std::string(argv[1])=="--delta") return differences();
    const int side=argc>1 ? std::stoi(argv[1]):100;
    const float spacing=argc>2 ? std::stof(argv[2]):.5f;
    const float spike=argc>3 ? std::stof(argv[3]):0;
    const bool newOnly=argc>4 && std::string(argv[4])=="--new-only";
    auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int y=0;y<side;++y) for (int x=0;x<side;++x)
        cloud->emplace_back(x*spacing,y*spacing,.01f*std::sin(x*.3f)*std::cos(y*.2f)+((y*side+x)%31==0 ? spike:0));
    double oldMs=-1;size_t oldAnomaly=0,oldInliers=0,oldOutliers=0;
    if (!newOnly) {
        BaselineFitPlane old(cloud,BaselineFitPlane::FitParams{});
        auto start=Clock::now();old.compute();oldMs=elapsed(start);
        oldAnomaly=old.Get_AnomalyPoints()->size();oldInliers=old.Get_Inliers()->size();oldOutliers=old.Get_Outliers()->size();
    }
    Fit_Plane current(cloud,Fit_Plane::FitParams{});
    auto start=Clock::now();current.compute();const double newMs=elapsed(start);
    std::cout<<std::fixed<<std::setprecision(3)<<"N="<<cloud->size()<<" spacing="<<spacing<<" spike="<<spike
        <<" old_full_ms="<<oldMs<<" new_full_ms="<<newMs<<" old_anomaly="<<oldAnomaly
        <<" old_inliers="<<oldInliers<<" old_outliers="<<oldOutliers
        <<" new_inliers="<<current.Get_Inliers()->size()<<" new_outliers="<<current.Get_Outliers()->size()<<std::endl;
    return current.Get_Coeff_in().size()==4 ? 0:1;
}
