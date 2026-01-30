#include "Fitting/Fit_Cylinder.h"

Fit_Cylinder::Fit_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr InputC) :
	paramDialog(new ParamDialog_FittingCylinder())
	, cloud_input(new pcl::PointCloud<pcl::PointXYZ>)
	, cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>)
	, cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>)
{
	bool ok1, ok2, ok3, ok4;  // 增加ok4用于初始半径验证

	if (paramDialog->exec() == QDialog::Accepted) // 如果用户点击了"确定"
	{
		QString param1 = paramDialog->getParams()[0];
		QString param2 = paramDialog->getParams()[1];
		QString param3 = paramDialog->getParams()[2];
		QString param4 = paramDialog->getParams()[3];  // 新增：获取初始半径参数
		KSearch = param1.toInt(&ok1);
		DistanceThreshold = param2.toFloat(&ok2);
		MaxIterations = param3.toInt(&ok3);
		InitialRadius = param4.toFloat(&ok4);  // 新增：解析初始半径
		if (!ok1 || !ok2 || !ok3 || !ok4) {  // 增加ok4检查
			qDebug() << "无效数字";
			return;
		}
	}
	else
	{
		qDebug() << "取消操作";
		return;
	}
	*cloud_input = *InputC;
	if (cloud_input->empty()) {
		qDebug() << "点云为空";
	}
	Proc();
}
Fit_Cylinder::~Fit_Cylinder() = default;

void Fit_Cylinder::Proc() {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setInputCloud(cloud_input);
	n.setSearchMethod(tree);
	n.setKSearch(KSearch);
	n.compute(*normals);

	pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >::Ptr model(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal >(cloud_input));
	model->setInputNormals(normals);

	// 新增：设置初始半径猜测
	model->setRadiusLimits(InitialRadius * 0.98, InitialRadius * 1.02);  // 设置半径搜索范围

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);	// 定义RANSAC算法对象

	ransac.setDistanceThreshold(DistanceThreshold);							// 设置距离阈值
	ransac.setMaxIterations(MaxIterations);								// 设置最大迭代次数
	ransac.computeModel();
	ransac.getModelCoefficients(coeff_in);							// 参数
	std::vector<int> ranSacInliers;                                                 // 获取属于拟合出的内点
	ransac.getInliers(ranSacInliers);
	pcl::copyPointCloud(*cloud_input, ranSacInliers, *cloud_inliers);
	ransac.computeModel();
	ransac.getModelCoefficients(coeff_in);

	// 添加安全检查
	if (coeff_in.size() < 7) {
		qDebug() << "圆柱拟合失败：未能获得有效的模型系数";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}

	// 检查半径是否有效
	if (coeff_in[6] <= 0) {
		qDebug() << "圆柱拟合失败：无效的半径值";
		cloud_inliers->clear();
		*cloud_outliers = *cloud_input;
		return;
	}

	qDebug() << "圆柱轴上一点的x坐标为：" << coeff_in[0] << "\n圆柱轴上一点的y坐标为：" << coeff_in[1] << "\n圆柱轴上一点的z坐标为：" << coeff_in[2]
		<< "\n圆柱轴方向的x为：" << coeff_in[3] << "\n圆柱轴方向的y为：" << coeff_in[4] << "\n圆柱轴方向的z为：" << coeff_in[5]
		<< "\n圆柱半径为：" << coeff_in[6];

	cloud_outliers->clear();
	std::unordered_set<int> inliers_set(ranSacInliers.begin(), ranSacInliers.end());
	for (size_t i = 0; i < cloud_input->size(); ++i) {
		if (!inliers_set.count(i)) {
			cloud_outliers->push_back((*cloud_input)[i]);
		}
	}

	// 新增：输出百分比信息
	qDebug() << "内点数量：" << cloud_inliers->size() << "，占总点数的：" << Get_Inliers_Percentage() << "%";
	qDebug() << "外点数量：" << cloud_outliers->size() << "，占总点数的：" << Get_Outliers_Percentage() << "%";
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
