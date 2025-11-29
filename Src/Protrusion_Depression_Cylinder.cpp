#include "Protrusion_Depression_Cylinder.h"
#include "Basic/Basic.h"
#include "CloudForgeAnalyzer.h"
#include <filesystem>
namespace fs = std::filesystem;

Protrusion_Depression_Cylinder::Protrusion_Depression_Cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr Source_cloud, std::function<void(const std::string&)> TeEDebug_callback):
	TeEDebug_callback(TeEDebug_callback)
	 // 初始化为包含10个nullptr的向量//定义了size，并不知道怎么灵活插入变量~-~
{
		Input_cloud = *Source_cloud;
		mainbody_fit_data="";
		// 初始化
}

Protrusion_Depression_Cylinder::~Protrusion_Depression_Cylinder()=default;

void Protrusion_Depression_Cylinder::VoxelGrid_Sor_filter()
{
	// 1. VoxelGrid滤波
	vg.setInputCloud(Input_cloud.makeShared()); // Input_cloud需为智能指针（假设是成员变量）
	TeEDebug_callback(">>BC_VSf:输入点云点数:" + std::to_string(Input_cloud.points.size()));
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(Vgfiltered_cloud);   // 直接操作栈对象，filter()内部使用引用

	// 3. StatisticalOutlierRemoval滤波
	sor.setInputCloud(Vgfiltered_cloud.makeShared()); // 栈对象转智能指针
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(Sored_cloud);       // 结果写入栈对象
	TeEDebug_callback(">>BC_fcM:预处理后点云点数:" + std::to_string(Sored_cloud.points.size()));
}
void Protrusion_Depression_Cylinder::FitCylinderModel()
{

	if (Sored_cloud.points.empty())
	{
		TeEDebug_callback(">>BC_fcM:输入点云为空");
		return;
	}

	//-----------------------------法线估计--------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;	// 创建法向量估计对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setSearchMethod(tree);						       // 设置搜索方式
	n.setInputCloud(Sored_cloud.makeShared());						           // 设置输入点云
	n.setKSearch(20);								       // 设置K近邻搜索点的个数
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);
	//----------------------------圆柱拟合--------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;// 创建圆柱体分割对象
	seg.setInputCloud(Sored_cloud.makeShared());					// 设置输入点云
	seg.setInputNormals(normals);								    // 设置输入法向量
	seg.setOptimizeCoefficients(true);								// 设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);						// 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);								// 设置采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.3);								// 设置表面法线权重系数
	seg.setMaxIterations(10000);									// 设置迭代的最大次数
	seg.setDistanceThreshold(0.1);									// 设置内点到模型距离的最大值
	seg.setRadiusLimits(1.0, 10.0);									// 设置圆柱模型半径的范围


	seg.segment(inliers_cylinder, coefficients_cylinder);	// 执行分割，将分割结果的索引保存到inliers_cylinder中，同时存储模型系数coefficients_cylinder

	if (inliers_cylinder.indices.size() == 0)
	{
		TeEDebug_callback(">>BC_fcM:并非圆柱.") ;
		// 提示用户
	}
	if (coefficients_cylinder.values[5] < 0) {
		coefficients_cylinder.values[5]=-coefficients_cylinder.values[5];
		coefficients_cylinder.values[4] = -coefficients_cylinder.values[4];
		coefficients_cylinder.values[3] = -coefficients_cylinder.values[3];
	}
	TeEDebug_callback(">>BC_fcM:A point on the cylindrical axis\n" + std::to_string('[') + std::to_string(coefficients_cylinder.values[0]) + ", " + std::to_string(coefficients_cylinder.values[1]) + ", " + std::to_string(coefficients_cylinder.values[2]) + std::to_string(']') + "\n");
	TeEDebug_callback(">>BC_fcM:The axial direction of the cylinder\n" + std::to_string('[') + std::to_string(coefficients_cylinder.values[3]) + ", " + std::to_string(coefficients_cylinder.values[4]) + ", " + std::to_string(coefficients_cylinder.values[5]) + std::to_string(']') + "\n");
	
	TeEDebug_callback(">>BC_fcM:Radius of the cylinder\n" + std::to_string('[') + std::to_string(coefficients_cylinder.values[3]) + std::to_string(']') + "\n");

	mainbody_fit_data="圆柱轴一点 \n["+ to_string_with_precision(coefficients_cylinder.values[0],4)+","+ to_string_with_precision(coefficients_cylinder.values[1], 4) +","+ to_string_with_precision(coefficients_cylinder.values[2], 4) +"]\n"
		+ "圆柱轴矢量\n["+ to_string_with_precision(coefficients_cylinder.values[3], 4) +","+ to_string_with_precision(coefficients_cylinder.values[4], 4) +","+ to_string_with_precision(coefficients_cylinder.values[5], 4) +"]\n"
		+ "圆柱半径\n"+ to_string_with_precision(coefficients_cylinder.values[6], 4) +"\n";
	
	//----------------------------主体和缺陷提取--------------------------------
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr defects(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointIndicesPtr inliers_cylinder_Ptr(new pcl::PointIndices);
	*inliers_cylinder_Ptr=inliers_cylinder;

	extract.setInputCloud(Sored_cloud.makeShared());
	extract.setIndices(inliers_cylinder_Ptr);
	extract.setNegative(false);
	extract.filter(*cylinder);


	extract.setNegative(true);
	extract.filter(*defects);

	pcl::PCDWriter writer;
	writer.write(path_cylinder, *cylinder, false);
	TeEDebug_callback(">>BC_fcM:生成主体点云"+path_cylinder);
	writer.write(path_defects, *defects, false);
	TeEDebug_callback(">>BC_fcM:生成缺陷点云" + path_defects);
	
	defects_cloud = defects;//直接扔给成员变量，避免二次读写文件
	main_cloud = cylinder;

	TeEDebug_callback(">>BC_fcM:Cloud now fit!");

}


std::string  Protrusion_Depression_Cylinder::get_mainbody_fit_data() {
	if (mainbody_fit_data != "") {
		return mainbody_fit_data;
	}
	return "";
}

std::string Protrusion_Depression_Cylinder::get_defect_fit_data() {
	if (defect_fit_data != "") {
		return defect_fit_data;
	}
	return "";
}

void Protrusion_Depression_Cylinder::AnalyzeDefects() {
	if (!defects_cloud || defects_cloud->empty()) {
		TeEDebug_callback(">>BC_aD: 无缺陷点云或请先分析主体");
		return;
	}
	defects_clouds.clear();
	defects_list.clear();
	_clusterDefects();
}


void Protrusion_Depression_Cylinder::MarkDefects(pcl::visualization::PCLVisualizer::Ptr viwerInput) {
	pcl::visualization::PCLVisualizer::Ptr viewer = viwerInput;
	int id = 1;
	for (const auto& defect : defects_list) {
		std::string text = std::to_string(id++);
		viewer->addText3D(text,//添加标记的方法，可以说非常之好用呢
			pcl::PointXYZ(defect.position.x, defect.position.y, defect.position.z),
			0.2, 1.0, 0.0, 0.0, "defect_" + text);
		TeEDebug_callback("defect_" + text + "最大深度:" + std::to_string(defect.max_depth));
	}
	id = 1;
	for (const auto& decloud : defects_clouds) {
		
	}
	TeEDebug_callback("defects_numbers:" + std::to_string(defects_list.size()));
}


void Protrusion_Depression_Cylinder::_clusterDefects() {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(defects_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.8); // 根据点云密度调整
	ec.setMinClusterSize(4);      // 最小点数过滤噪声
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(defects_cloud);
	ec.extract(cluster_indices);

	int defect_id = 1;
	for (const auto& cluster : cluster_indices) {
		pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
		pcl::copyPointCloud(*defects_cloud, cluster, cluster_cloud);
		defects_clouds.push_back(cluster_cloud);
		_calculateDepth(cluster_cloud);
		defects_list.back().id = defect_id++; // 记录序号
	}
	defect_id = 1;
	defect_fit_data = "";

	for (const auto& defect : defects_list) {
		std::string text = std::to_string(defect_id++);
		defect_fit_data += "defect_" + text + "深度:" + std::to_string(defect.max_depth) + "\n";
	}//记录缺陷数据

	TeEDebug_callback(">>Bc_cD:" + defect_fit_data);

}

void Protrusion_Depression_Cylinder::_calculateDepth(const pcl::PointCloud<pcl::PointXYZ>& cluster) {
	float max_depth = 0;
	pcl::PointXYZ max_point;

	// 提取圆柱参数
	const auto& coeff = coefficients_cylinder.values;
	Eigen::Vector3f axis_point(coeff[0], coeff[1], coeff[2]);
	Eigen::Vector3f axis_dir(coeff[3], coeff[4], coeff[5]);
	float radius = coeff[6];

	for (const auto& point : cluster.points) {
		Eigen::Vector3f pt(point.x, point.y, point.z);
		Eigen::Vector3f vec = pt - axis_point;
		float projection = vec.dot(axis_dir); // 沿轴方向投影
		Eigen::Vector3f pt_proj = axis_point + projection * axis_dir;
		float distance = (pt - pt_proj).norm() - radius; // 点到圆柱的距离

		if (fabs(distance) > fabs(max_depth)) {
			max_depth = distance;
			max_point = point;
		}
	}

	defects_list.push_back({ 0, max_depth, max_point }); // id由外层填充
}