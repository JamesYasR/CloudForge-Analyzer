// 无界面探针: 完全复刻 GUI 路径(固定轴线/不拟合)在给定两份点云上跑一次焊前测量
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "Measure/MeasureWeldPreparation.h"
int main(int argc, char** argv) {
    if (argc < 5) { std::cerr << "用法: probe_pair <far.pcd> <near.pcd> <R> <ux,uy,uz>\n"; return 2; }
    auto far = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto near = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(argv[1], *far) < 0) { std::cerr << "读 far 失败\n"; return 1; }
    if (pcl::io::loadPCDFile(argv[2], *near) < 0) { std::cerr << "读 near 失败\n"; return 1; }
    std::cout << "far=" << far->size() << " near=" << near->size() << std::endl;
    const double R = std::atof(argv[3]);
    double x, y, z; sscanf(argv[4], "%lf,%lf,%lf", &x, &y, &z);
    Eigen::Vector3d u(x, y, z); u.normalize();
    std::cout << "R=" << R << " axis=" << u.transpose() << std::endl;

    MeasureWeldPreparation mp;
    mp.setNearCloud(near, "near");
    mp.setFarCloud(far, "far");
    MeasureWeldPreparation::Params p;
    p.design_radius = R;
    p.metric = MeasureWeldPreparation::Metric::Both;
    p.gap_direction = MeasureWeldPreparation::GapDirection::Axial;
    mp.setParams(p);
    mp.setFixedReferenceAxis(Eigen::Vector3d::Zero(), u);
    mp.setRoleOverride(MeasureWeldPreparation::Role::Near, MeasureWeldPreparation::Role::Far, "probe");
    mp.setProgressCallback([](int c, int t, const std::string& s) {
        std::cout << "[PROG " << c << "/" << t << "] " << s << std::endl; return true; });
    auto r = mp.evaluate();
    std::cout << "STATUS=" << static_cast<int>(r.status) << std::endl;
    std::cout << r.report << std::endl;
    return 0;
}
