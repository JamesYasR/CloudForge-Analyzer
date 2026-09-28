#pragma once
#include "headers.h"
#include "Basic/ColorManager.h"

class Cluster {
public:
	// 新增：聚类参数(用于后台计算: 参数已在界面线程收集完毕, 不再弹对话框)
	struct Params {
		float tolerance = 0.02f;      // 聚类容差
		float minSize = 100.0f;       // 最小聚类点数
		float maxSize = 5000000.0f;   // 最大聚类点数
	};

	Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c);
	// 新增：已知参数构造(不弹参数对话框, 不自动计算) —— 配合 compute() 在工作线程中执行
	Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c, const Params& params);
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> GetClusterMap() const;
	std::map<int, ColorManager> GetColorMap() const;
	// 新增：执行欧式聚类计算(可被工作线程调用)
	void compute();
	bool isCancelled = false;   // 新增：是否被取消(原有构造函数的取消/无效参数分支会置位)
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