
#include "Linear_Depression_Plane.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

Linear_Depression_Plane::Linear_Depression_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr Source_cloud, std::function<void(const std::string&)> TeEDebug_callback):
	TeEDebug_callback(TeEDebug_callback) 
{
	Input_cloud = *Source_cloud;
}
Linear_Depression_Plane::~Linear_Depression_Plane() = default;

void Linear_Depression_Plane::VoxelGrid_Sor_filter() {
	vg.setInputCloud(Input_cloud.makeShared()); // Input_cloud需为智能指针（假设是成员变量）
	TeEDebug_callback(">>LDP_VSf:输入点云点数:" + std::to_string(Input_cloud.points.size()));
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(Vgfiltered_cloud); 
	sor.setInputCloud(Vgfiltered_cloud.makeShared()); // 栈对象转智能指针
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(Sored_cloud);       // 结果写入栈对象
}
#include "Linear_Depression_Plane.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

void Linear_Depression_Plane::FitPlaneModel() {
    if (Sored_cloud.points.empty()) {
        TeEDebug_callback(">>FitPlaneModel: cloud is empty");
        return;
    }

    // 创建平面分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01); // 根据实际场景调整阈值

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 执行平面分割
    seg.setInputCloud(Sored_cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        TeEDebug_callback(">>FitPlaneModel: plane fit failed");
        return;
    }

    // 保存平面模型参数
    coefficients_plane = *coefficients;
    inliers_plane.indices = inliers->indices;

    // 输出平面特征值（平面方程系数）
    TeEDebug_callback(">>FitPlaneModel: Plane Pra: a=" + std::to_string(coefficients->values[0]) +
        ", b=" + std::to_string(coefficients->values[1]) +
        ", c=" + std::to_string(coefficients->values[2]) +
        ", d=" + std::to_string(coefficients->values[3]));
    mainbody_fit_data = ">>FitPlaneModel: Plane Pra: a=" + std::to_string(coefficients->values[0]) +
        ", b=" + std::to_string(coefficients->values[1]) +
        ", c=" + std::to_string(coefficients->values[2]) +
        ", d=" + std::to_string(coefficients->values[3]);
    //TeEDebug_callback(">>FitPlaneModel: 内点数量: " + std::to_string(inliers->indices.size()));
    //TeEDebug_callback(">>FitPlaneModel: 内点表示输入点云中符合平面模型的点的索引集合");

    // 提取平面主点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(Sored_cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    main_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*main_cloud);

    // 提取凹陷候选点云
    extract.setNegative(true);
    defects_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*defects_cloud);

    pcl::PCDWriter writer;
    writer.write(path_plane, *main_cloud, false);
    TeEDebug_callback(">>BC_fcM:generate main_cloud" + path_plane);
    writer.write(path_defects, *defects_cloud, false);
    TeEDebug_callback(">>BC_fcM:generate defect_cloud" + path_defects);
    // 输出调试信息
    TeEDebug_callback(">>FitPlaneModel: main count: " + std::to_string(main_cloud->size()));
    TeEDebug_callback(">>FitPlaneModel: defect count: " + std::to_string(defects_cloud->size()));
}
void Linear_Depression_Plane::MarkDefects(pcl::visualization::PCLVisualizer::Ptr viwerInput) {}
void Linear_Depression_Plane::AnalyzeDefects(){}

std::string Linear_Depression_Plane::get_mainbody_fit_data() {
    return mainbody_fit_data;
}
std::string Linear_Depression_Plane::get_defect_fit_data() {
    return defect_fit_data;
}