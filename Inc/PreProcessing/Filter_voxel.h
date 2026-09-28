#pragma once
#include "headers.h"

class Filter_voxel {
public:
	// 新增：滤波参数(用于后台计算: 参数已在界面线程收集完毕, 不再弹对话框)
	struct Params {
		float leafsize = 0.2f;   // 体素尺寸(叶大小)
	};

	Filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c);
	// 新增：已知参数构造(不弹参数对话框, 不自动计算) —— 配合 compute() 在工作线程中执行
	Filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c, const Params& params);
	~Filter_voxel();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Get_filtered();
	// 新增：执行体素滤波计算(可被工作线程调用)
	void compute();
	bool isCancelled = false;   // 新增：是否被取消(原有构造函数的取消/无效参数分支会置位)
private:
	float vg_size;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output_cloud;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	void Proc();
	ParamDialog_vg* paramDialog;
};

