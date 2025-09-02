#include "preprocessing/ProtrusionSegmentation.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/point_tests.h> // 用于pcl::isFinite检查
#include <Eigen/Dense>

ProtrusionSegmentation::ProtrusionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    : input_cloud(input_cloud)
    , planar_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , protrusion_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , search_radius(0.03f)
    , height_threshold(0.01f)
    , min_cluster_size(100)
    , dialog(new ParamDialogProtrusion())
{
    bool ok1 = false, ok2 = false, ok3 = false;
    if (dialog->exec() == QDialog::Accepted) {
        height_threshold = dialog->getParams()[0].toFloat(&ok1);
        search_radius = dialog->getParams()[1].toFloat(&ok2);
        min_cluster_size = dialog->getParams()[2].toInt(&ok3);

        if (ok1 && ok2 && ok3) {
            qDebug() << "有效参数:" << height_threshold << search_radius << min_cluster_size;
            is_valid = true;
        }
        else {
            qDebug() << "无效参数";
            is_valid = false;
        }
    }
    else {
        qDebug() << "操作取消";
        is_valid = false;
    }
}

void ProtrusionSegmentation::segment() {
    if (!is_valid) {
        qDebug() << "分割器状态无效，跳过处理";
        return;
    }

    planar_cloud->clear();
    protrusion_cloud->clear();

    // 局部拟合判断突起点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);
    pcl::PointIndices::Ptr protrusion_indices(new pcl::PointIndices);

    for (size_t i = 0; i < input_cloud->size(); ++i) {
        const auto& query = input_cloud->points[i];
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.radiusSearch(query, search_radius, indices, dists);

        if (indices.size() < 10) continue; // 邻域点太少跳过

        // 局部平面拟合 z = ax + by + c
        Eigen::MatrixXf A(indices.size(), 3);
        Eigen::VectorXf b(indices.size());
        for (size_t j = 0; j < indices.size(); ++j) {
            const auto& pt = input_cloud->points[indices[j]];
            A(j, 0) = pt.x;
            A(j, 1) = pt.y;
            A(j, 2) = 1.0;
            b(j) = pt.z;
        }
        Eigen::Vector3f coeff = A.colPivHouseholderQr().solve(b);

        float z_fit = coeff[0]*query.x + coeff[1]*query.y + coeff[2];

        float dist = query.z - z_fit;
        float abs_dist = std::abs(query.z - z_fit);
        if (abs_dist > height_threshold) {
            protrusion_indices->indices.push_back(i);
        }
    }

    // 3. 提取候选突起区域
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    extract.setInputCloud(input_cloud);
    extract.setIndices(protrusion_indices);
    extract.setNegative(false);
    extract.filter(*candidate_cloud);

    // 4. 欧几里得聚类过滤小噪声
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ec_tree->setInputCloud(candidate_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2.0 * search_radius);
    ec.setMinClusterSize(min_cluster_size);
    ec.setSearchMethod(ec_tree);
    ec.setInputCloud(candidate_cloud);
    ec.extract(cluster_indices);

    // 5. 合并有效聚类并创建有效索引
    pcl::PointIndices::Ptr valid_indices(new pcl::PointIndices);
    for (const auto& cluster : cluster_indices) {
        for (const auto& idx : cluster.indices) {
            protrusion_cloud->points.push_back(candidate_cloud->points[idx]);
            valid_indices->indices.push_back(protrusion_indices->indices[idx]);
        }
    }

    // 6. 提取平面区域（总点云 - 有效突起点）
    extract.setInputCloud(input_cloud);
    extract.setIndices(valid_indices);
    extract.setNegative(true);
    extract.filter(*planar_cloud);

    qDebug() << "处理完成 - 平面点数量:" << planar_cloud->size()
        << " 突起点数量:" << protrusion_cloud->size();
}