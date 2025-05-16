#pragma once
#include "headers.h"

class Filter_sor {
public:
	Filter_sor(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c);
	~Filter_sor();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Get_filtered();
private:
	int sor_mean_k;
	float sor_std_dev_mul_thresh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output_cloud;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	void Proc();
	ParamDialog_sor* paramDialog;
};