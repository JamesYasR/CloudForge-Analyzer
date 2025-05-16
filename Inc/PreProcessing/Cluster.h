#pragma once
#include "headers.h"
#include "ColorManager.h"

class Cluster {
public:
	Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c);
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> GetClusterMap() const;
	std::map<int, ColorManager> GetColorMap() const;
private:
	float tolerance, min, max;
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;//ktree搜索
	pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output_cloud;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_map; // 存储编号和点云
	std::map<int, ColorManager> color_map; // 存储编号和颜色
	void Proc();
	ParamDialog_ec* paramDialog;

};