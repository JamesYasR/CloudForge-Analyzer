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
struct defect_info_1 {
    int id;             // 缺陷序号
    float max_depth;    // 最大深度（正为凸，负为凹）
    pcl::PointXYZ position; // 最大深度点的坐标
};
//PCLINCLUDEEND
class Protrusion_Depression_Cylinder
{
	public:
        Protrusion_Depression_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr Source_cloud, std::function<void(const std::string&)> TeEDebug_callback);
        ~Protrusion_Depression_Cylinder();
        void MarkDefects(pcl::visualization::PCLVisualizer::Ptr viwerInput);
        void VoxelGrid_Sor_filter();
        void FitCylinderModel();//拟合圆柱模型
        void AnalyzeDefects();

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        pcl::PointCloud<pcl::PointXYZ> Input_cloud;
        pcl::PointCloud<pcl::PointXYZ> Vgfiltered_cloud;
        pcl::PointCloud<pcl::PointXYZ> Sored_cloud;

        pcl::PointIndices inliers_cylinder;	// 保存分割结果
        pcl::ModelCoefficients coefficients_cylinder;	// 保存圆柱体模型系数
        std::function<void(const std::string&)> TeEDebug_callback;

        //pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGrid_Sor_filter();//下采样，体素滤波


        std::string get_mainbody_fit_data();
        std::string get_defect_fit_data();
        std::string mainbody_fit_data;
        std::string defect_fit_data;


        std::vector<defect_info_1> defects_list; // 存储缺陷信息


        pcl::PointCloud<pcl::PointXYZ>::Ptr defects_cloud; // 直接存储缺陷点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud;
    private:
        std::string path_cylinder = "PCDfiles/split/cylinder.pcd";
        std::string path_defects = "PCDfiles/split/defects.pcd";
        std::vector<pcl::PointCloud<pcl::PointXYZ>> defects_clouds;
        void _clusterDefects(); // 聚类缺陷点云
        void _calculateDepth(const pcl::PointCloud<pcl::PointXYZ>& cluster); // 计算单个缺陷深度

};