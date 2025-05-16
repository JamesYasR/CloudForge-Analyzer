#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>  
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include "Dialog/Dialog.h"
#include <unordered_set>

class Fit_Cylinder {
public:
	Fit_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC);
	~Fit_Cylinder();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Inliers();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Get_Outliers();
	Eigen::VectorXf Get_Coeff_in();
	float ComputeCylinderHeight();
private:
	int KSearch;
	float DistanceThreshold;
	int MaxIterations;
	Eigen::VectorXf coeff_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;
	ParamDialog_FittingCylinder* paramDialog;
	void Proc();

};