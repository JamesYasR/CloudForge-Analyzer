#include "PreProcessing/Cluster.h"
#include <omp.h>

Cluster::Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_c) :
	Input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	, paramDialog(new ParamDialog_ec())
	, tree(new pcl::search::KdTree<pcl::PointXYZ>)
{
	bool ok1, ok2, ok3;
	if (paramDialog->exec() == QDialog::Accepted) // 如果用户点击了“确定”
	{
		QString param3 = paramDialog->getParams()[2];
		QString param2 = paramDialog->getParams()[1];
		QString param1 = paramDialog->getParams()[0]; // 获取输入的参数
		tolerance = param1.toFloat(&ok1);
		min = param2.toFloat(&ok2);
		max = param3.toFloat(&ok3);
		if (!ok1 || !ok2 || !ok3) {
			qDebug() << "无效数字";
			return;
		}
	}
	else
	{
		qDebug() << "取消操作";
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

	int num_clusters = cluster_indices.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp_clusters(num_clusters);
	std::vector<ColorManager> temp_colors(num_clusters);

#pragma omp parallel for
	for (int cluster_id = 0; cluster_id < num_clusters; ++cluster_id) {
		const auto& indices = cluster_indices[cluster_id];
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

		// 提取点云
		cloud_cluster->reserve(indices.indices.size());
		for (const auto& idx : indices.indices) {
			cloud_cluster->points.push_back((*Input_cloud)[idx]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// 生成颜色（使用线程安全的方式）
		int r = (cluster_id * 137) % 256;  // 使用确定性颜色，避免rand()竞争
		int g = (cluster_id * 193) % 256;
		int b = (cluster_id * 257) % 256;

		temp_clusters[cluster_id] = cloud_cluster;
		temp_colors[cluster_id] = ColorManager(r, g, b);
	}

	// 串行写入结果（避免map竞争）
	for (int cluster_id = 0; cluster_id < num_clusters; ++cluster_id) {
		cluster_map.emplace(cluster_id, temp_clusters[cluster_id]);
		color_map.emplace(cluster_id, temp_colors[cluster_id]);
	}

	qDebug() << "总聚类数：" << num_clusters;

}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> Cluster::GetClusterMap() const {
	return cluster_map;
}

std::map<int, ColorManager> Cluster::GetColorMap() const {
	return color_map;
}
