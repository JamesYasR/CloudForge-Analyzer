#include "CloudForgeAnalyzer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    CloudForgeAnalyzer win;
    win.show();
    return app.exec();
}
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>
//#include <random>
//#include <cmath>
//
//// ���ɴ���ͻ���ƽ�����
//pcl::PointCloud<pcl::PointXYZ>::Ptr generatePlaneWithProtrusion(
//    float width = 1.0f,
//    float height = 1.0f,
//    float protrusion_height = 0.03f,
//    float protrusion_width = 0.3f,
//    float protrusion_height_ratio = 0.8f,
//    float noise_level = 0.002f,
//    float point_spacing = 0.005f)
//{
//    // ��������
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // �������
//    int width_points = static_cast<int>(width / point_spacing);
//    int height_points = static_cast<int>(height / point_spacing);
//
//    // ����������������������
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::normal_distribution<float> dist(0.0f, noise_level);
//
//    // ͻ��Ĳ�����ƽ���������һ������ͻ��
//    float protrusion_center_x = width / 2.0f;
//    float protrusion_center_y = height / 2.0f;
//    float protrusion_half_width = protrusion_width / 2.0f;
//    float protrusion_half_height = protrusion_height / 2.0f * protrusion_height_ratio;
//
//    // ����ƽ�����
//    for (int w = 0; w < width_points; ++w) {
//        float x = static_cast<float>(w) / width_points * width;
//
//        for (int h = 0; h < height_points; ++h) {
//            float y = static_cast<float>(h) / height_points * height;
//            float z = 0.0f; // ����ƽ��߶�
//
//            // ����Ƿ���ͻ��������
//            float dx = x - protrusion_center_x;
//            float dy = y - protrusion_center_y;
//
//            // ����ͻ������
//            if (std::abs(dx) < protrusion_half_width && std::abs(dy) < protrusion_half_height) {
//                // ����ͻ����״��������ߣ���Եƽ�����ɣ�
//                float x_factor = 1.0f - std::pow(dx / protrusion_half_width, 2);
//                float y_factor = 1.0f - std::pow(dy / protrusion_half_height, 2);
//                float protrusion = protrusion_height * x_factor * y_factor;
//                z += protrusion;
//            }
//
//            // ����������
//            x += dist(gen);
//            y += dist(gen);
//            z += dist(gen);
//
//            cloud->points.push_back(pcl::PointXYZ(x, y, z));
//        }
//    }
//
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = false;
//
//    return cloud;
//}
//
//// �����˲�����
//pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(
//    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
//    float leaf_size = 0.01f)
//{
//    pcl::VoxelGrid<pcl::PointXYZ> filter;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
//
//    filter.setInputCloud(input_cloud);
//    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
//    filter.filter(*filtered);
//
//    return filtered;
//}
//
//int main()
//{
//    // ���ɴ�ͻ���ƽ�����
//    auto raw_cloud = generatePlaneWithProtrusion(
//        1.0f,     // ƽ���� 1��
//        1.0f,     // ƽ��߶� 1��
//        0.03f,    // ͻ��߶� 3����
//        0.3f,     // ͻ���� 30����
//        0.5f,     // ͻ��߶ȱ� (�߶�/���)
//        0.002f,   // ����ˮƽ 2����
//        0.002f    // ���� 2����
//    );
//
//    //// ����ԭʼ����
//    //pcl::io::savePCDFile("plane_with_protrusion_raw.pcd", *raw_cloud);
//    //std::cout << "done " << raw_cloud->size() << " points" << std::endl;
//
//    // Ӧ�������˲� (0.01m)
//    auto filtered_cloud = voxelFilter(raw_cloud, 0.01f);
//
//    // �����˲������
//    pcl::io::savePCDFile("plane_with_protrusion_filtered.pcd", *filtered_cloud);
//    std::cout << "done " << filtered_cloud->size() << " points" << std::endl;
//
//    return 0;
//}