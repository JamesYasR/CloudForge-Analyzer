//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/transforms.h>
//#include <iostream>
//#include <random>
//#include <cmath>
//#include <vector>
//
//// ================== 可调参数 ==================
//// 平面尺寸参数
//const double PLANE_WIDTH = 100.0;      // 平面宽度 (mm)
//const double PLANE_HEIGHT = 100.0;     // 平面高度 (mm)
//const double POINT_SPACING = 1.0;      // 点间距 (mm)
//
//// 随机凹坑/突起参数
//const int NUM_DEFECTS = 20;            // 缺陷数量
//const double MAX_DEFECT_HEIGHT = 1.2;  // 最大缺陷高度 (mm)
//const double MIN_DEFECT_HEIGHT = 0.2;  // 最小缺陷高度 (mm)
//const double MIN_DEFECT_RADIUS = 2.0;  // 最小缺陷半径 (mm)
//const double MAX_DEFECT_RADIUS = 8.0;  // 最大缺陷半径 (mm)
//
//// 高斯噪声参数
//const double GAUSSIAN_NOISE_STD = 0.05; // 高斯噪声标准差 (mm)
//
//// 输出文件
//const std::string OUTPUT_FILENAME = "test.pcd";
//// ==============================================
//
//// 2D高斯函数
//double gaussian2D(double x, double y, double center_x, double center_y, double sigma) {
//    double dx = x - center_x;
//    double dy = y - center_y;
//    double exponent = -(dx * dx + dy * dy) / (2.0 * sigma * sigma);
//    return std::exp(exponent);
//}
//
//// 生成缺陷结构
//struct Defect {
//    double center_x;  // 中心X坐标
//    double center_y;  // 中心Y坐标
//    double amplitude; // 振幅（正为突起，负为凹坑）
//    double radius;    // 影响半径
//    Defect(double x, double y, double a, double r)
//        : center_x(x), center_y(y), amplitude(a), radius(r) {
//    }
//};
//
//int main() {
//    // 初始化随机数生成器
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<> pos_x_dist(0.0, PLANE_WIDTH);
//    std::uniform_real_distribution<> pos_y_dist(0.0, PLANE_HEIGHT);
//    std::uniform_real_distribution<> height_dist(MIN_DEFECT_HEIGHT, MAX_DEFECT_HEIGHT);
//    std::uniform_real_distribution<> radius_dist(MIN_DEFECT_RADIUS, MAX_DEFECT_RADIUS);
//    std::uniform_real_distribution<> sign_dist(-1.0, 1.0); // 随机正负
//    std::normal_distribution<> gaussian_noise(0.0, GAUSSIAN_NOISE_STD);
//
//    // 创建点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 生成网格点
//    int num_points_x = static_cast<int>(PLANE_WIDTH / POINT_SPACING) + 1;
//    int num_points_y = static_cast<int>(PLANE_HEIGHT / POINT_SPACING) + 1;
//
//    std::cout << "生成平面点云..." << std::endl;
//    std::cout << "平面尺寸: " << PLANE_WIDTH << "mm x " << PLANE_HEIGHT << "mm" << std::endl;
//    std::cout << "点间距: " << POINT_SPACING << "mm" << std::endl;
//    std::cout << "网格点数: " << num_points_x << " x " << num_points_y << " = "
//        << num_points_x * num_points_y << std::endl;
//    std::cout << "缺陷数量: " << NUM_DEFECTS << std::endl;
//    std::cout << "最大缺陷高度: " << MAX_DEFECT_HEIGHT << "mm" << std::endl;
//    std::cout << "高斯噪声标准差: " << GAUSSIAN_NOISE_STD << "mm" << std::endl;
//
//    // 生成随机缺陷
//    std::vector<Defect> defects;
//    for (int i = 0; i < NUM_DEFECTS; ++i) {
//        double x = pos_x_dist(gen);
//        double y = pos_y_dist(gen);
//        double height = height_dist(gen);
//
//        // 随机决定是突起还是凹坑
//        double amplitude = height;
//        if (sign_dist(gen) < 0) {
//            amplitude = -amplitude; // 凹坑
//        }
//
//        double radius = radius_dist(gen);
//        defects.emplace_back(x, y, amplitude, radius);
//
//        std::cout << "缺陷 " << i + 1 << ": 中心(" << x << ", " << y
//            << "), 振幅=" << amplitude << "mm, 半径=" << radius << "mm" << std::endl;
//    }
//
//    // 生成点云
//    for (int i = 0; i < num_points_x; ++i) {
//        for (int j = 0; j < num_points_y; ++j) {
//            double x = i * POINT_SPACING;
//            double y = j * POINT_SPACING;
//            double z = 0.0; // 基础平面高度
//
//            // 计算所有缺陷对该点的影响
//            for (const auto& defect : defects) {
//                double dx = x - defect.center_x;
//                double dy = y - defect.center_y;
//                double distance = std::sqrt(dx * dx + dy * dy);
//
//                if (distance <= defect.radius) {
//                    // 使用高斯函数来模拟平滑的缺陷
//                    double weight = std::exp(-(distance * distance) / (2.0 * defect.radius * defect.radius));
//                    z += defect.amplitude * weight;
//                }
//            }
//
//            // 添加高斯噪声
//            z += gaussian_noise(gen);
//
//            cloud->points.emplace_back(x, y, z);
//        }
//    }
//
//    // 设置点云属性
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    // 统计信息
//    double min_z = std::numeric_limits<double>::max();
//    double max_z = std::numeric_limits<double>::lowest();
//    double sum_z = 0.0;
//
//    for (const auto& point : cloud->points) {
//        if (point.z < min_z) min_z = point.z;
//        if (point.z > max_z) max_z = point.z;
//        sum_z += point.z;
//    }
//
//    double avg_z = sum_z / cloud->points.size();
//    double pv = max_z - min_z; // 峰谷值
//
//    std::cout << "\n点云统计信息:" << std::endl;
//    std::cout << "最小Z值: " << min_z << " mm" << std::endl;
//    std::cout << "最大Z值: " << max_z << " mm" << std::endl;
//    std::cout << "平均Z值: " << avg_z << " mm" << std::endl;
//    std::cout << "峰谷值(PV): " << pv << " mm" << std::endl;
//    std::cout << "总点数: " << cloud->points.size() << std::endl;
//
//    // 可选：添加离群点（模拟真实噪声）
//    std::uniform_real_distribution<> outlier_x_dist(0.0, PLANE_WIDTH);
//    std::uniform_real_distribution<> outlier_y_dist(0.0, PLANE_HEIGHT);
//    std::uniform_real_distribution<> outlier_z_dist(-5.0, 5.0); // 离群点高度范围
//
//    int num_outliers = static_cast<int>(cloud->points.size() * 0.01); // 1%的离群点
//    for (int i = 0; i < num_outliers; ++i) {
//        cloud->points.emplace_back(
//            outlier_x_dist(gen),
//            outlier_y_dist(gen),
//            outlier_z_dist(gen)
//        );
//    }
//
//    std::cout << "添加了 " << num_outliers << " 个离群点" << std::endl;
//
//    // 重新设置点云属性（添加离群点后）
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    // 验证点云属性
//    std::cout << "\n点云属性验证:" << std::endl;
//    std::cout << "width * height = " << cloud->width << " * " << cloud->height
//        << " = " << cloud->width * cloud->height << std::endl;
//    std::cout << "实际点数: " << cloud->points.size() << std::endl;
//
//    if (cloud->width * cloud->height != cloud->points.size()) {
//        std::cerr << "错误: width * height 不等于点云数量!" << std::endl;
//        std::cerr << "修正为 width = 点数, height = 1" << std::endl;
//        // 修正为无序点云的格式
//        cloud->width = cloud->points.size();
//        cloud->height = 1;
//    }
//
//    // 保存点云
//    std::cout << "\n正在保存点云到: " << OUTPUT_FILENAME << std::endl;
//    try {
//        pcl::io::savePCDFileASCII(OUTPUT_FILENAME, *cloud);
//        std::cout << "点云已成功保存" << std::endl;
//    }
//    catch (const std::exception& e) {
//        std::cerr << "保存PCD文件时出错: " << e.what() << std::endl;
//
//        // 尝试使用二进制格式保存
//        std::cout << "尝试使用二进制格式保存..." << std::endl;
//        pcl::io::savePCDFileBinary(OUTPUT_FILENAME, *cloud);
//        std::cout << "二进制格式保存成功" << std::endl;
//    }
//
//    // 验证文件保存
//    std::cout << "\n验证文件加载..." << std::endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(OUTPUT_FILENAME, *loaded_cloud) == 0) {
//        std::cout << "验证: 文件成功加载，包含 " << loaded_cloud->points.size() << " 个点" << std::endl;
//        std::cout << "width: " << loaded_cloud->width << ", height: " << loaded_cloud->height << std::endl;
//    }
//    else {
//        std::cout << "警告: 文件加载验证失败" << std::endl;
//    }
//
//    return 0;
//}