#include "PreProcessing/Filter_voxel.h"

Filter_voxel::Filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c) :
	Input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, Output_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, paramDialog(new ParamDialog_vg())
{
	bool ok;
	float leafsize;
	if (paramDialog->exec() == QDialog::Accepted) // 如果用户点击了“确定”
	{
		QString param = paramDialog->getParams()[0]; // 获取输入的参数
		leafsize = param.toFloat(&ok);
		if (!ok) {
			qDebug() << "无效数字";
			isCancelled = true;
			return;
		}
		qDebug() << "参数：" << param;
	}
	else
	{
		qDebug() << "取消操作";
		isCancelled = true;
		return;
	}
	Input_cloud = Input_c;
	vg_size = leafsize;
	Proc();
}

Filter_voxel::Filter_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c, const Params& params)
	: Input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, Output_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, paramDialog(nullptr)
{
	// 参数由界面线程收集, 此处仅保存输入(不弹框、不计算)
	Input_cloud = Input_c;
	vg_size = params.leafsize;
}

void Filter_voxel::compute() {
	Proc();
}

Filter_voxel::~Filter_voxel() = default;

void Filter_voxel::Proc() {
	vg.setInputCloud(Input_cloud);
	vg.setLeafSize(vg_size, vg_size, vg_size);
	vg.filter(*Output_cloud);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter_voxel::Get_filtered() {
	return Output_cloud;
}

