#include "PreProcessing/Cluster.h"


Cluster::Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c) :
	Input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, paramDialog(new ParamDialog_ec())
	, tree(new pcl::search::KdTree<pcl::PointXYZ>)
{
	bool ok1, ok2, ok3;
	if (paramDialog->exec() == QDialog::Accepted) // 如果用户点击了“确定”
	{
		QString param3 = paramDialog->getParam()[2];
		QString param2 = paramDialog->getParam()[1];
		QString param1 = paramDialog->getParam()[0]; // 获取输入的参数
		tolerance = param1.toFloat(&ok1);
		min = param2.toFloat(&ok2);
		max = param3.toFloat(&ok3);
		if (!ok1 || !ok2 || !ok3) {
			qDebug() << "无效数字";
			return;
		}
		//qDebug() << "参数："  << param1 << "," << param2 << "," << param3;
		//qDebug() << "tolerance:" << tolerance << "min:" << min << "max:" << max;
	}
	else
	{
		//qDebug() << "取消操作";
		return;
	}
	*Input_cloud = *Input_c;
	//qDebug() << "DIAN:" + QString::number(Input_cloud->size());
	Proc();
}
void Cluster::Proc() {
	tree->setInputCloud(Input_cloud);
	ec.setClusterTolerance(tolerance); // 2cm
	ec.setMinClusterSize(min);
	ec.setMaxClusterSize(max);
	ec.setSearchMethod(tree);
	ec.setInputCloud(Input_cloud);
	//聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
	ec.extract(cluster_indices);

	int cluster_id = 0;
	for (const auto& indices : cluster_indices) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& idx : indices.indices) {
			cloud_cluster->points.push_back((*Input_cloud)[idx]); // 添加点
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// 生成随机颜色
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		qDebug() << r << g << b;
		ColorManager color(r,g,b);
		// 创建颜色处理器
		color_map.emplace(
			cluster_id,color
		);

		// 存储点云（同理处理cluster_map，若需要）
		cluster_map.emplace(cluster_id, cloud_cluster);


		cluster_id++;
	}

	//qDebug() << "总聚类数：" << cluster_id;

}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::GetClusterMap() const {
	return cluster_map;
}

std::map<int, ColorManager> Cluster::GetColorMap() const {
	return color_map;
}
