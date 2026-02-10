//#include "Basic/CloudClipper.h"
//#include "config/pcl114.h"
//int main() {
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile<pcl::PointXYZ>("PCDfiles/rabbit.pcd", *cloud);
//	interactivePolygonCut(cloud);
//}

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <cmath>
//
//int main() {
//    // 创建点云对象
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 圆柱参数
//    const float radius = 2000.0f;      // 半径
//    const float height = 100.0f;       // 总高度
//    const float angle_deg = 10.0f;     // 圆弧角度（度）
//    const int angle_resolution = 100;  // 角度方向采样点数
//    const int height_resolution = 50;  // 高度方向采样点数
//
//    // 将角度转换为弧度，以0度为中心，向两边各延伸5度
//    const float angle_rad = angle_deg * M_PI / 180.0f;
//    const float start_angle = -angle_rad / 2.0f;
//    const float end_angle = angle_rad / 2.0f;
//
//    // 生成圆柱面点云（轴线沿X轴）
//    // 参数方程：y = r*cos(θ), z = r*sin(θ), x = [-height/2, height/2]
//    for (int i = 0; i <= height_resolution; ++i) {
//        float x = -height / 2.0f + (height * i / height_resolution);
//
//        for (int j = 0; j <= angle_resolution; ++j) {
//            float angle = start_angle + (end_angle - start_angle) * j / angle_resolution;
//
//            pcl::PointXYZ point;
//            point.x = x;
//            point.y = radius * std::cos(angle);  // Y方向
//            point.z = radius * std::sin(angle);  // Z方向
//
//            cloud->points.push_back(point);
//        }
//    }
//
//    // 设置点云属性
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    // 保存为PCD文件
//    pcl::io::savePCDFileASCII("cylinder_partial.pcd", *cloud);
//
//    std::cout << "成功生成点云，共 " << cloud->points.size() << " 个点" << std::endl;
//    std::cout << "参数信息：" << std::endl;
//    std::cout << "  - 中心: (0, 0, 0)" << std::endl;
//    std::cout << "  - 轴线方向: (1, 0, 0)" << std::endl;
//    std::cout << "  - 半径: " << radius << std::endl;
//    std::cout << "  - 高度: " << height << std::endl;
//    std::cout << "  - 圆弧角度: " << angle_deg << "度" << std::endl;
//    std::cout << "  - X范围: [" << -height / 2 << ", " << height / 2 << "]" << std::endl;
//    std::cout << "  - 角度范围: [" << start_angle * 180 / M_PI << ", " << end_angle * 180 / M_PI << "]度" << std::endl;
//
//    return 0;
//}