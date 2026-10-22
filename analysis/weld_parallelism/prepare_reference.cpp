#include "Fitting/Fit_Cylinder.h"
#include "Measure/MeasureCylindricity.h"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc,char** argv)
{
    if (argc!=3) return 2;
    auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(argv[1],*cloud)<0) return 3;
    const auto start=std::chrono::steady_clock::now();
    Fit_Cylinder::FitParams params;params.InitialRadius=1940;
    Fit_Cylinder initial(cloud,params);initial.compute();
    const auto first=initial.Get_Coeff_in();
    if (first.size()!=7) return 4;
    MeasureCylindricity fit;
    fit.setInputCloud(cloud);fit.setDesignRadius(1940);fit.setTolerance(1);fit.setMaxIterations(2000);fit.setVerbose(false);
    fit.setInitialLine(initial.get_center_point(),initial.get_axis_direction());
    fit.setWeldThresholdFactor(3);fit.setWeldClusterTolerance(2);fit.setWeldMinClusterSize(20);fit.setWeldConstraintWeight(1);
    std::string last;
    fit.setProgressCallback([&](int current,int total,const std::string& stage) {
        if (last!=stage) { std::cout<<"[STAGE] "<<stage<<" "<<current<<"/"<<total<<std::endl;last=stage; }
        return true;
    });
    auto result=fit.evaluateCylindricityWithWeld();
    if (fit.isCancelled()) return 5;
    const auto point=result.getCylinderAxisPoint(),direction=result.getCylinderAxisDirection();
    if (!point.allFinite() || !direction.allFinite() || direction.norm()<.5) return 6;
    std::ofstream out(argv[2]);
    out<<std::setprecision(17);
    out<<"points "<<cloud->size()<<"\nradius 1940\ninitial";
    for (int i=0;i<7;++i) out<<' '<<first[i];
    out<<"\nfinal "<<point.x()<<' '<<point.y()<<' '<<point.z()<<' '<<direction.x()<<' '<<direction.y()<<' '<<direction.z()<<" 1940\n";
    out<<"weld_constraint "<<fit.isWeldConstraintApplied()<<"\nweld_points "<<fit.getWeldPointCount()<<"\nrms "<<result.rms_error<<'\n';
    out<<"seconds "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
    std::cout<<"[REFERENCE] "<<argv[2]<<" weld_constraint="<<fit.isWeldConstraintApplied()<<std::endl;
    return out ? 0:7;
}
