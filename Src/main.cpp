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
//// 生成带有突起的平面点云
//pcl::PointCloud<pcl::PointXYZ>::Ptr generatePlaneWithProtrusion(
//    float width = 1.0f,
//    float height = 1.0f,
//    float protrusion_height = 0.03f,
//    float protrusion_width = 0.3f,
//    float protrusion_height_ratio = 0.8f,
//    float noise_level = 0.002f,
//    float point_spacing = 0.005f)
//{
//    // 创建点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 计算点数
//    int width_points = static_cast<int>(width / point_spacing);
//    int height_points = static_cast<int>(height / point_spacing);
//
//    // 随机数生成器用于添加噪声
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::normal_distribution<float> dist(0.0f, noise_level);
//
//    // 突起的参数（平面中心添加一个矩形突起）
//    float protrusion_center_x = width / 2.0f;
//    float protrusion_center_y = height / 2.0f;
//    float protrusion_half_width = protrusion_width / 2.0f;
//    float protrusion_half_height = protrusion_height / 2.0f * protrusion_height_ratio;
//
//    // 生成平面点云
//    for (int w = 0; w < width_points; ++w) {
//        float x = static_cast<float>(w) / width_points * width;
//
//        for (int h = 0; h < height_points; ++h) {
//            float y = static_cast<float>(h) / height_points * height;
//            float z = 0.0f; // 基础平面高度
//
//            // 检查是否在突起区域内
//            float dx = x - protrusion_center_x;
//            float dy = y - protrusion_center_y;
//
//            // 矩形突起区域
//            if (std::abs(dx) < protrusion_half_width && std::abs(dy) < protrusion_half_height) {
//                // 计算突起形状（中心最高，边缘平滑过渡）
//                float x_factor = 1.0f - std::pow(dx / protrusion_half_width, 2);
//                float y_factor = 1.0f - std::pow(dy / protrusion_half_height, 2);
//                float protrusion = protrusion_height * x_factor * y_factor;
//                z += protrusion;
//            }
//
//            // 添加随机噪声
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
//// 体素滤波函数
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
//    // 生成带突起的平面点云
//    auto raw_cloud = generatePlaneWithProtrusion(
//        1.0f,     // 平面宽度 1米
//        1.0f,     // 平面高度 1米
//        0.03f,    // 突起高度 3厘米
//        0.3f,     // 突起宽度 30厘米
//        0.5f,     // 突起高度比 (高度/宽度)
//        0.002f,   // 噪声水平 2毫米
//        0.002f    // 点间距 2毫米
//    );
//
//    //// 保存原始点云
//    //pcl::io::savePCDFile("plane_with_protrusion_raw.pcd", *raw_cloud);
//    //std::cout << "done " << raw_cloud->size() << " points" << std::endl;
//
//    // 应用体素滤波 (0.01m)
//    auto filtered_cloud = voxelFilter(raw_cloud, 0.01f);
//
//    // 保存滤波后点云
//    pcl::io::savePCDFile("plane_with_protrusion_filtered.pcd", *filtered_cloud);
//    std::cout << "done " << filtered_cloud->size() << " points" << std::endl;
//
//    return 0;
//}