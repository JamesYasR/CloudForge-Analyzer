#include "PreProcessing/Filter_sor.h"

Filter_sor::Filter_sor(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c) :
	Input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, Output_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, paramDialog(new ParamDialog_sor())
{
	bool ok1, ok2;
	int mean_k;
	float std_dev_mul_thresh;
	if (paramDialog->exec() == QDialog::Accepted) // 如果用户点击了“确定”
	{
		QString param1 = paramDialog->getParams()[0]; // 获取输入的参数
		QString param2 = paramDialog->getParams()[1]; // 获取输入的参数
		mean_k = param1.toInt(&ok1);
		std_dev_mul_thresh = param2.toFloat(&ok2);
		if (!ok1 && !ok2) {
			qDebug() << "无效数字";
			return;
		}
		qDebug() << "参数：" << param1 << param2;
	}
	else
	{
		qDebug() << "取消操作";
		return;
	}
	Input_cloud = Input_c;
	sor_mean_k = mean_k;
	sor_std_dev_mul_thresh = std_dev_mul_thresh;
	Proc();
}
Filter_sor::~Filter_sor() = default;

void Filter_sor::Proc() {
	sor.setInputCloud(Input_cloud);
	sor.setMeanK(sor_mean_k);
	sor.setStddevMulThresh(sor_std_dev_mul_thresh);
	sor.filter(*Output_cloud);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter_sor::Get_filtered() {
	return Output_cloud;
}