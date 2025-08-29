#include "preprocessing/CurvatureSegmentation.h"
CurvatureSegmentation::CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) :
	input_cloud(input_cloud)
    ,planar_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , weld_cloud(new pcl::PointCloud<pcl::PointXYZ>) 
    , curvature_threshold(0.02f)
    , normal_radius(0.03f)
	, min_cluster_size(100) 
	, dialog(new ParamDialogCurvSeg())
{
    bool ok1, ok2,ok3;
    float curvature_threshold_;
    float normal_radius_;
    int min_cluster_size_;
    if (dialog->exec() == QDialog::Accepted) // 如果用户点击了“确定”
    {
        curvature_threshold_ = dialog->getParams()[0].toFloat(&ok1);
        normal_radius_ = dialog->getParams()[1].toFloat(&ok2);
        min_cluster_size_ = dialog->getParams()[2].toInt(&ok3);
        if (!ok1 && !ok2 && !ok3) {
            qDebug() << "无效数字";
            return;
        }
        min_cluster_size = min_cluster_size_;
        curvature_threshold = curvature_threshold_;
        normal_radius = normal_radius_;
        qDebug() << "参数：" << curvature_threshold_ << normal_radius_ << min_cluster_size_;
    }
    else
    {
        qDebug() << "取消操作";
        return;
    }

}
void CurvatureSegmentation::segment() {
    // 估计点云法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(input_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);

    // 准备曲率索引
    pcl::IndicesPtr high_curvature_indices(new std::vector<int>);
    pcl::IndicesPtr low_curvature_indices(new std::vector<int>);

    // 曲率阈值分割
    for (size_t i = 0; i < normals->size(); ++i) {
        if (normals->at(i).curvature > curvature_threshold) {
            high_curvature_indices->push_back(i);  // 焊缝区域
        }
        else {
            low_curvature_indices->push_back(i);   // 平面区域
        }
    }

    // 提取点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);

    // 提取焊缝点云
    extract.setIndices(high_curvature_indices);
    extract.setNegative(false);
    extract.filter(*weld_cloud);

    // 提取平面点云
    extract.setIndices(low_curvature_indices);
    extract.filter(*planar_cloud);
}