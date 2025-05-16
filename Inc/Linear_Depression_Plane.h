#pragma once

#include <memory>
#include <regex>
#include <string>
#include <vector>

//QINCLUDEHEAD
#include "QString"
#include "QDebug"
//QINCLUDEEND
//PCLINCLUDEHEAD
#include <Boost/smart_ptr/make_shared.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>	
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
struct defect_info_2 {
    int id;             // 缺陷序号
    double max_depth;    // 最大深度（正为凸，负为凹）
    double ave_depth;    //平均深度
    double straightness; //计算出直线度
    pcl::ModelCoefficients line_coeff;//记录缺陷拟合直线的特征
};
//PCLINCLUDEEND
class Linear_Depression_Plane
{
public:
    Linear_Depression_Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr Source_cloud, std::function<void(const std::string&)> TeEDebug_callback);
    ~Linear_Depression_Plane();
    void MarkDefects(pcl::visualization::PCLVisualizer::Ptr viwerInput);
    void VoxelGrid_Sor_filter();
    void FitPlaneModel();//拟合圆柱模型
    void AnalyzeDefects();

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ> Input_cloud;
    pcl::PointCloud<pcl::PointXYZ> Vgfiltered_cloud;
    pcl::PointCloud<pcl::PointXYZ> Sored_cloud;

    pcl::PointIndices inliers_plane;	// 保存平面分割结果
    pcl::ModelCoefficients coefficients_plane;	
    std::function<void(const std::string&)> TeEDebug_callback;


    std::string get_mainbody_fit_data();
    std::string get_defect_fit_data();
    std::string mainbody_fit_data;
    std::string defect_fit_data;

    std::vector<defect_info_2> defects_list; // 存储缺陷信息

    pcl::PointCloud<pcl::PointXYZ>::Ptr defects_cloud; // 直接存储缺陷点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud;
private:
    std::string path_plane = "PCDfiles/split/plane.pcd";
    std::string path_defects = "PCDfiles/split/defects_plane.pcd";
    std::vector<pcl::PointCloud<pcl::PointXYZ>> defects_clouds;
    void _clusterDefects(); // 聚类缺陷点云
    void _calculateDefects(const pcl::PointCloud<pcl::PointXYZ>& cluster); // 在这里经过计算填充defect_info计算缺陷最大深度，平均深度，在这里记录直线凹陷的位置，
};