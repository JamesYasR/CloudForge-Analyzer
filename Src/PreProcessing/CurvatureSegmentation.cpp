#include "PreProcessing/CurvatureSegmentation.h"
#include <unordered_set>
#include <pcl/common/common.h> 
#include <mutex> // 添加此头文件以修复 std::mutex 未定义的问题
#include <pcl/features/normal_3d_omp.h> // 添加头文件以解决 NormalEstimationOMP 未定义的问题
#include <pcl/segmentation/sac_segmentation.h> // 添加此头文件以修复 SACSegmentation 未定义的问题
#include <pcl/filters/voxel_grid.h>
#include <queue>
CurvatureSegmentation::CurvatureSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointXYZ input_point) :
    input_cloud(input_cloud)
	, output_cloud(new pcl::PointCloud<pcl::PointXYZ>())
    , dialog(new ParamDialogCurvSeg())
{
    picked_point = input_point;
    bool ok1, ok2, ok3, ok4;
    int  k_search_;
    float smooth_threshold_;
    float curvature_threshold_;
    int min_cluster_size_;

    if (dialog->exec() == QDialog::Accepted) {
        k_search_ = dialog->getParams()[0].toInt(&ok1);
        smooth_threshold_ = dialog->getParams()[1].toFloat(&ok2);
        curvature_threshold_ = dialog->getParams()[2].toFloat(&ok3);
        min_cluster_size_ = dialog->getParams()[3].toInt(&ok4);

        if (!ok1 || !ok2 || !ok3 || !ok4 ) {
            qDebug() << "无效数字";
            return;
        }

        k_search = k_search_;
        smooth_threshold = smooth_threshold_;
        curvature_threshold = curvature_threshold_;
        min_cluster_size = min_cluster_size_;
    }
    else {
        qDebug() << "取消操作";
        return;
    }
    extractPlane();
}

void CurvatureSegmentation::extractPlane() {
    if (input_cloud->empty()) {
        std::cerr << "输入点云为空！" << std::endl;
        return;
    }

    // 0. 下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    // 1. 计算法线和曲率
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(k_search);
    normal_estimator.compute(*normals);

    // 2. 找到种子点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_filtered);
    std::vector<int> seed_idx(1);
    std::vector<float> seed_dist(1);

    if (kdtree.nearestKSearch(picked_point, 1, seed_idx, seed_dist) > 0) {
        int seed_index = seed_idx[0];

        // 3. 区域生长算法改进
        std::vector<bool> processed(cloud_filtered->size(), false);
        std::vector<int> cluster_indices;
        std::queue<int> seed_queue;

        seed_queue.push(seed_index);
        processed[seed_index] = true;

        // 计算种子点的法线，作为平面的参考法线
        Eigen::Vector3f seed_normal = normals->points[seed_index].getNormalVector3fMap();

        // 统计聚类信息，用于动态调整阈值
        std::vector<Eigen::Vector3f> cluster_normals;
        std::vector<float> cluster_curvatures;

        // 记录种子点信息
        cluster_normals.push_back(seed_normal);
        cluster_curvatures.push_back(normals->points[seed_index].curvature);

        while (!seed_queue.empty()) {
            int current_idx = seed_queue.front();
            seed_queue.pop();

            cluster_indices.push_back(current_idx);

            // 搜索当前点的邻域
            std::vector<int> neighbors;
            std::vector<float> distances;
            kdtree.nearestKSearch(cloud_filtered->points[current_idx], k_search, neighbors, distances);

            for (int neighbor_idx : neighbors) {
                if (processed[neighbor_idx]) continue;

                // 获取邻居点的法线和曲率
                Eigen::Vector3f neighbor_normal = normals->points[neighbor_idx].getNormalVector3fMap();
                float curvature = normals->points[neighbor_idx].curvature;

                // 关键改进：只与种子点法线对比，不需要与当前点法线对比
                // 这样可以确保整个平面都是基于同一个参考法线
                float angle_to_seed = acosf(seed_normal.dot(neighbor_normal));

                // 判断条件：曲率小（平坦区域）且法线方向一致
                if (curvature < curvature_threshold &&
                    angle_to_seed < smooth_threshold) {

                    // 进一步检查局部连续性
                    bool local_consistent = true;

                    // 检查该点与其周围已处理点的法线一致性（可选，增强鲁棒性）
                    for (int i = 0; i < 5 && i < cluster_normals.size(); ++i) {
                        float local_angle = acosf(neighbor_normal.dot(cluster_normals[i]));
                        if (local_angle > smooth_threshold * 2.0f) { // 局部检查更严格
                            local_consistent = false;
                            break;
                        }
                    }

                    if (local_consistent) {
                        processed[neighbor_idx] = true;
                        seed_queue.push(neighbor_idx);
                        cluster_normals.push_back(neighbor_normal);
                        cluster_curvatures.push_back(curvature);
                    }
                }
            }
        }

        // 4. 提取聚类点云
        if (cluster_indices.size() >= min_cluster_size) {
            output_cloud->clear();

            // 使用KDTree从原始点云中获取对应点
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_original;
            kdtree_original.setInputCloud(input_cloud);

            // 先收集所有滤波后的点
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (int idx : cluster_indices) {
                filtered_cluster->push_back(cloud_filtered->points[idx]);
            }

            // 使用半径搜索从原始点云中查找更多点
            for (const auto& point : *filtered_cluster) {
                std::vector<int> indices;
                std::vector<float> distances;

                // 使用较小的半径搜索，确保找到对应点
                if (kdtree_original.radiusSearch(point, 0.02f, indices, distances) > 0) {
                    for (int idx : indices) {
                        output_cloud->push_back(input_cloud->points[idx]);
                    }
                }
            }

            // 可选：对输出点云进行去重
            std::sort(output_cloud->begin(), output_cloud->end(),
                [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                    if (a.x != b.x) return a.x < b.x;
                    if (a.y != b.y) return a.y < b.y;
                    return a.z < b.z;
                });
            output_cloud->erase(std::unique(output_cloud->begin(), output_cloud->end(),
                [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                    return fabs(a.x - b.x) < 0.001f &&
                        fabs(a.y - b.y) < 0.001f &&
                        fabs(a.z - b.z) < 0.001f;
                }), output_cloud->end());

            std::cout << "成功提取平面，点数: " << output_cloud->size() << std::endl;
        }
        else {
            std::cerr << "提取的平面点数量(" << cluster_indices.size()
                << ")小于最小聚类大小(" << min_cluster_size << ")" << std::endl;
        }
    }
    else {
        std::cerr << "未找到输入点的最近邻点！" << std::endl;
    }
}