#include "Fitting/Fit_Cylinder.h"
#include <omp.h>
#include <algorithm>
#include <chrono>
#include <QElapsedTimer>

namespace {
static QElapsedTimer g_fitTimer;
#define FIT_LOG(msg) do { \
    if (!g_fitTimer.isValid()) g_fitTimer.start(); \
    qDebug().noquote() << "[FitCyl]" << msg << "+" << g_fitTimer.restart() << "ms"; \
} while(0)
}

Fit_Cylinder::Fit_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC) :
	dialog(new ParamDialog_FittingCylinder())
	, cloud_input(new pcl::PointCloud<pcl::PointXYZ>)
	, cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>)
	, cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>)
{
	bool ok1, ok2, ok3, ok4;  // 增加ok4用于初始半径验证
	if (dialog->exec() != QDialog::Accepted) {
		isCancelled = true;
		return;
	}
	else // 如果用户点击了"确定"
	{
		QString param1 = dialog->getParams()[0];
		QString param2 = dialog->getParams()[1];
		QString param3 = dialog->getParams()[2];
		QString param4 = dialog->getParams()[3];  // 新增：获取初始半径参数
		KSearch = param1.toInt(&ok1);
		DistanceThreshold = param2.toFloat(&ok2);
		MaxIterations = param3.toInt(&ok3);
		InitialRadius = param4.toFloat(&ok4);  // 新增：解析初始半径
		if (!ok1 || !ok2 || !ok3 || !ok4) {  // 增加ok4检查
			qDebug() << "无效数字";
			return;
		}
	}
	*cloud_input = *InputC;
	if (cloud_input->empty()) {
		qDebug() << "点云为空";
	}
	Proc();
}
Fit_Cylinder::~Fit_Cylinder() = default;

void Fit_Cylinder::Proc() {
	if (!cloud_input || cloud_input->empty()) {
		qDebug() << "点云为空，无法进行圆柱拟合";
		return;
	}

	// 法线估计：限制线程数，避免 Windows Release 下首次 OpenMP 线程池初始化导致卡顿/异常
	const int hardware_threads = omp_get_max_threads();
	const int normal_threads = std::max(1, std::min(hardware_threads, 4));
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setNumberOfThreads(normal_threads);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setInputCloud(cloud_input);
	n.setSearchMethod(tree);
	n.setKSearch(KSearch);
	FIT_LOG("kdtree+setInput 完成, 开始法线估计");
	n.compute(*normals);
	FIT_LOG(QString("法线估计完成(线程=%1)").arg(normal_threads));
	if (normals->size() != cloud_input->size()) {
		qDebug() << "法线估计失败：法线数量与输入点云不一致";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}

	pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >::Ptr model(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >(cloud_input));
	model->setInputNormals(normals);

	// 新增：设置初始半径猜测
	if (InitialRadius > 0.0) {
		model->setRadiusLimits(InitialRadius * 0.998, InitialRadius * 1.002);  // 设置半径搜索范围
	}

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);	// 定义RANSAC算法对象
	ransac.setDistanceThreshold(DistanceThreshold);							// 设置距离阈值
	ransac.setMaxIterations(MaxIterations);								// 设置最大迭代次数

	// RANSAC 只执行一次，避免重复计算和状态不一致
	FIT_LOG("模型/RANSAC对象就绪, 开始computeModel");
	if (!ransac.computeModel()) {
		qDebug() << "RANSAC 圆柱拟合失败：未找到有效模型";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}
	ransac.getModelCoefficients(coeff_in);							// 参数
	std::vector<int> ranSacInliers;                                                 // 获取属于拟合出的内点
	ransac.getInliers(ranSacInliers);
	FIT_LOG(QString("RANSAC完成 内点=%1").arg(ranSacInliers.size()));
	pcl::copyPointCloud(*cloud_input, ranSacInliers, *cloud_inliers);
	FIT_LOG("copyPointCloud内点完成");

	// 添加安全检查
	if (coeff_in.size() < 7) {
		qDebug() << "圆柱拟合失败：未能获得有效的模型系数";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}

	// 检查半径是否有效
	if (coeff_in[6] <= 0 || !std::isfinite(coeff_in[6])) {
		qDebug() << "圆柱拟合失败：无效的半径值";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}

	qDebug() << "圆柱轴上一点的x坐标为：" << coeff_in[0] << "\n圆柱轴上一点的y坐标为：" << coeff_in[1] << "\n圆柱轴上一点的z坐标为：" << coeff_in[2]
		<< "\n圆柱轴方向的x为：" << coeff_in[3] << "\n圆柱轴方向的y为：" << coeff_in[4] << "\n圆柱轴方向的z为：" << coeff_in[5]
		<< "\n圆柱半径为：" << coeff_in[6];

	// 外点提取：使用 O(N) 标记法替代原来的 O(N*M) std::find，避免点云较大时卡顿
	cloud_outliers->clear();
	const int total_points = static_cast<int>(cloud_input->size());
	cloud_outliers->reserve(total_points - static_cast<int>(ranSacInliers.size()));

	std::vector<char> is_inlier(total_points, 0);
	for (int idx : ranSacInliers) {
		if (idx >= 0 && idx < total_points) {
			is_inlier[idx] = 1;
		}
	}

#pragma omp parallel
	{
		std::vector<pcl::PointXYZ> local_outliers;
#pragma omp for nowait
		for (int i = 0; i < total_points; ++i) {
			if (!is_inlier[i]) {
				local_outliers.push_back((*cloud_input)[i]);
			}
		}

#pragma omp critical
		{
			cloud_outliers->insert(cloud_outliers->end(), local_outliers.begin(), local_outliers.end());
		}
	}
	FIT_LOG(QString("外点提取完成 外点=%1").arg(cloud_outliers->size()));

	// 新增：输出百分比信息
	qDebug() << "内点数量：" << cloud_inliers->size() << "，占总点数的：" << Get_Inliers_Percentage() << "%";
	qDebug() << "外点数量：" << cloud_outliers->size() << "，占总点数的：" << Get_Outliers_Percentage() << "%";


	message = "圆柱1轴上一点坐标为：" + std::to_string(coeff_in[0]) + ", " + std::to_string(coeff_in[1]) + ", " + std::to_string(coeff_in[2])
		+ "\n圆柱轴方向为：" + std::to_string(coeff_in[3]) + "," + std::to_string(coeff_in[4]) + "," + std::to_string(coeff_in[5])
		+ "\n圆柱半径为：" + std::to_string(coeff_in[6])
		+ "\n内点比例为：" + std::to_string(Get_Inliers_Percentage())
		+ "\nRMSE：" + std::to_string(ComputeRMSE());

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Cylinder::Get_Inliers() {
	return cloud_inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fit_Cylinder::Get_Outliers() {
	return cloud_outliers;
}

Eigen::VectorXf Fit_Cylinder::Get_Coeff_in() {
	return coeff_in;
}

float Fit_Cylinder::ComputeCylinderHeight() {
	if (coeff_in.size() < 7) {
		qDebug() << "模型系数不足，无法计算圆柱高度";
		return -1.0f;
	}

	Eigen::Vector3f axis(coeff_in[3], coeff_in[4], coeff_in[5]); // 圆柱轴向向量
	Eigen::Vector3f center(coeff_in[0], coeff_in[1], coeff_in[2]); // 圆柱轴上一点

	float min_height = std::numeric_limits<float>::max();
	float max_height = std::numeric_limits<float>::lowest();

	for (const auto& point : *cloud_inliers) {
		Eigen::Vector3f point_vec(point.x, point.y, point.z);
		float projection = (point_vec - center).dot(axis.normalized()); // 计算点在轴向上的投影值
		if (projection < min_height) min_height = projection;
		if (projection > max_height) max_height = projection;
	}

	float height = max_height - min_height;
	qDebug() << "圆柱高度为：" << height;
	return height;
}

// 新增：计算内点百分比
float Fit_Cylinder::Get_Inliers_Percentage() {
	if (cloud_input->empty()) {
		return 0.0f;
	}
	return (static_cast<float>(cloud_inliers->size()) / static_cast<float>(cloud_input->size())) * 100.0f;
}

// 新增：计算外点百分比
float Fit_Cylinder::Get_Outliers_Percentage() {
	if (cloud_input->empty()) {
		return 0.0f;
	}
	return (static_cast<float>(cloud_outliers->size()) / static_cast<float>(cloud_input->size())) * 100.0f;
}


Eigen::Vector3f Fit_Cylinder::get_center_point() {
	return Eigen::Vector3f(coeff_in[0], coeff_in[1], coeff_in[2]);
}

Eigen::Vector3f Fit_Cylinder::get_axis_direction() {
	return Eigen::Vector3f(coeff_in[3], coeff_in[4], coeff_in[5]);
}

float Fit_Cylinder::ComputeRMSE() {
	if (coeff_in.size() < 7 || cloud_inliers->empty()) {
		return -1.0f;
	}

	Eigen::Vector3f axis(coeff_in[3], coeff_in[4], coeff_in[5]);
	Eigen::Vector3f center(coeff_in[0], coeff_in[1], coeff_in[2]);
	float radius = coeff_in[6];

	float total_squared_error = 0.0f;
	int num_points = cloud_inliers->size();

#pragma omp parallel for reduction(+:total_squared_error)
	for (int i = 0; i < num_points; ++i) {
		const auto& point = cloud_inliers->points[i];
		Eigen::Vector3f point_vec(point.x, point.y, point.z);

		Eigen::Vector3f vec_to_center = point_vec - center;
		float projection_length = vec_to_center.dot(axis.normalized());
		Eigen::Vector3f projection_point = center + projection_length * axis.normalized();
		float distance_to_axis = (point_vec - projection_point).norm();

		float radial_error = std::abs(distance_to_axis - radius);
		total_squared_error += radial_error * radial_error;
	}

	return std::sqrt(total_squared_error / num_points);
}