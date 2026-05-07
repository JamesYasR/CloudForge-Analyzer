#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include "Dialog/ParamDialogCurvSeg.h"

class CurvatureSegmentation {
public:

    CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointXYZ input_point);

    std::string message;
	pcl::PointCloud<pcl::PointXYZ>::Ptr getOutputCloud() const { return output_cloud; }

private:
    int  k_search;
    float smooth_threshold;
    float curvature_threshold;
    int min_cluster_size;
	ParamDialogCurvSeg* dialog; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;
	pcl::PointXYZ picked_point;

    void extractPlane();

};