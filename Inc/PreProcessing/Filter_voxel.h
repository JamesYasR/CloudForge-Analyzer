#pragma once
#include "headers.h"

class Filter_voxel {
public:
	Filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c);
	~Filter_voxel();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Get_filtered();
private:
	float vg_size;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output_cloud;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	void Proc();
	ParamDialog_vg* paramDialog;
};

