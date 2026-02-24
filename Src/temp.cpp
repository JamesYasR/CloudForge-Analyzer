//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <iostream>
//#include <cmath>
//#include <map>
//#include <algorithm>
//
//// 定义点类型
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//// 九宫格位置枚举（二维定义）
//enum GridPosition {
//    TOP_LEFT, TOP_CENTER, TOP_RIGHT,
//    MIDDLE_LEFT, MIDDLE_CENTER, MIDDLE_RIGHT,
//    BOTTOM_LEFT, BOTTOM_CENTER, BOTTOM_RIGHT
//};
//
//// 位置到坐标的映射（角度，高度偏移）
//struct PositionCoord {
//    double angle;        // 周向位置（弧度）
//    double height_ratio; // 高度位置（-1到1，-1为底部，1为顶部）
//};
//
//std::map<GridPosition, PositionCoord> positionToCoord = {
//    // 顶部行
//    {TOP_LEFT,     { -M_PI / 4,  0.7 }},  // -45度，高度70%位置
//    {TOP_CENTER,   { 0.0,      0.7 }},  // 0度，高度70%位置
//    {TOP_RIGHT,    { M_PI / 4,   0.7 }},  // 45度，高度70%位置
//
//    // 中间行
//    {MIDDLE_LEFT,  { -M_PI / 2,  0.0 }},  // -90度，高度中心
//    {MIDDLE_CENTER,{ 0.0,      0.0 }},  // 0度，高度中心
//    {MIDDLE_RIGHT, { M_PI / 2,   0.0 }},  // 90度，高度中心
//
//    // 底部行
//    {BOTTOM_LEFT,  { -3 * M_PI / 4, -0.7 }}, // -135度，高度-70%位置
//    {BOTTOM_CENTER,{ M_PI,      -0.7 }}, // 180度，高度-70%位置
//    {BOTTOM_RIGHT, { 3 * M_PI / 4,  -0.7 }}  // 135度，高度-70%位置
//};
//
//// 生成二维局部变形的圆柱点云
//PointCloudT::Ptr generate2DLocalDeformedCylinder(
//    double radius,                // 基础半径
//    double height,                // 圆柱高度
//    double start_angle,           // 起始角度（弧度）
//    double end_angle,             // 结束角度（弧度）
//    double max_deformation,       // 最大变形量（径向向内）
//    GridPosition deform_position, // 变形位置
//    double angle_range = 0.05,    // 周向变形范围（弧度）
//    double height_range = 0.3     // 高度变形范围（高度比例）
//) {
//    PointCloudT::Ptr cloud(new PointCloudT);
//
//    // 分辨率设置
//    double angular_resolution = 0.002; // 约0.1度
//    double height_resolution = 0.02;    // 0.5mm
//
//    // 获取变形中心坐标
//    PositionCoord center = positionToCoord[deform_position];
//    double deform_angle = center.angle;
//    double deform_height = center.height_ratio * (height / 2.0); // 转换为实际高度
//
//    // 确保变形角度在有效范围内
//    deform_angle = std::max(deform_angle, start_angle);
//    deform_angle = std::min(deform_angle, end_angle);
//
//    // 计算变形范围边界
//    double angle_start = deform_angle - angle_range;
//    double angle_end = deform_angle + angle_range;
//    double height_start = deform_height - height_range * (height / 2.0);
//    double height_end = deform_height + height_range * (height / 2.0);
//
//    // 限制在有效范围内
//    angle_start = std::max(angle_start, start_angle);
//    angle_end = std::min(angle_end, end_angle);
//    height_start = std::max(height_start, -height / 2.0);
//    height_end = std::min(height_end, height / 2.0);
//
//    std::cout << "二维变形范围:" << std::endl;
//    std::cout << "周向: " << angle_start * 180 / M_PI << "° 到 " << angle_end * 180 / M_PI << "°" << std::endl;
//    std::cout << "高度: " << height_start << "mm 到 " << height_end << "mm" << std::endl;
//
//    int num_angles = static_cast<int>((end_angle - start_angle) / angular_resolution) + 1;
//    int num_heights = static_cast<int>(height / height_resolution) + 1;
//
//    for (int i = 0; i < num_angles; ++i) {
//        double angle = start_angle + i * angular_resolution;
//
//        for (int j = 0; j < num_heights; ++j) {
//            double y = j * height_resolution - height / 2.0; // 当前点高度
//
//            double deformation = 0.0;
//
//            // 检查是否在二维变形范围内
//            bool in_angle_range = (angle >= angle_start && angle <= angle_end);
//            bool in_height_range = (y >= height_start && y <= height_end);
//
//            if (in_angle_range && in_height_range) {
//                // 计算周向权重
//                double norm_angle = (angle - deform_angle) / angle_range;
//                double angle_weight = std::cos(norm_angle * M_PI / 2.0);
//                angle_weight = std::max(0.0, angle_weight);
//
//                // 计算高度权重
//                double norm_height = (y - deform_height) / (height_range * height / 2.0);
//                double height_weight = std::cos(norm_height * M_PI / 2.0);
//                height_weight = std::max(0.0, height_weight);
//
//                // 综合权重
//                double combined_weight = angle_weight * height_weight;
//                deformation = max_deformation * combined_weight;
//            }
//
//            // 应用变形
//            double effective_radius = radius - deformation;
//            double x = effective_radius * std::cos(angle);
//            double z = effective_radius * std::sin(angle);
//
//            PointT point;
//            point.x = x;
//            point.y = y;
//            point.z = z;
//
//            cloud->points.push_back(point);
//        }
//    }
//
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    return cloud;
//}
//
//// 辅助函数
//std::string positionToString(GridPosition pos) {
//    switch (pos) {
//    case TOP_LEFT: return "top_left";
//    case TOP_CENTER: return "top_center";
//    case TOP_RIGHT: return "top_right";
//    case MIDDLE_LEFT: return "middle_left";
//    case MIDDLE_CENTER: return "middle_center";
//    case MIDDLE_RIGHT: return "middle_right";
//    case BOTTOM_LEFT: return "bottom_left";
//    case BOTTOM_CENTER: return "bottom_center";
//    case BOTTOM_RIGHT: return "bottom_right";
//    default: return "unknown";
//    }
//}
//
//int main(int argc, char** argv) {
//    // 基本参数
//    double radius = 2000.0;
//    double height = 100.0;
//    double start_angle = -M_PI / 72;  // -5度
//    double end_angle = M_PI / 72;     // +5度
//    double max_deformation = 1.2;
//
//    // 二维变形范围参数
//    double angle_range = 0.03;  // 周向范围约1.7度
//    double height_range = 0.4;  // 高度范围比例（40%高度）
//
//    // 测试所有九宫格位置
//    std::vector<GridPosition> all_positions = {
//        TOP_LEFT, TOP_CENTER, TOP_RIGHT,
//        MIDDLE_LEFT, MIDDLE_CENTER, MIDDLE_RIGHT,
//        BOTTOM_LEFT, BOTTOM_CENTER, BOTTOM_RIGHT
//    };
//
//    for (auto position : all_positions) {
//        std::cout << "\n========================================" << std::endl;
//        std::cout << "生成位置: " << positionToString(position) << std::endl;
//
//        PointCloudT::Ptr deformed_cloud = generate2DLocalDeformedCylinder(
//            radius, height, start_angle, end_angle,
//            max_deformation, position, angle_range, height_range);
//
//        // 统计变形点
//        int deformed_points = 0;
//        for (const auto& p : deformed_cloud->points) {
//            double actual_radius = std::sqrt(p.x * p.x + p.z * p.z);
//            if (std::abs(actual_radius - radius) > 0.1) {
//                deformed_points++;
//            }
//        }
//
//        std::cout << "变形点数: " << deformed_points << "/" << deformed_cloud->size()
//            << " (" << 100.0 * deformed_points / deformed_cloud->size() << "%)" << std::endl;
//
//        // 保存文件
//        std::string filename = "test/2d_deformed_" + positionToString(position) + ".pcd";
//        pcl::io::savePCDFileASCII(filename, *deformed_cloud);
//        std::cout << "已保存: " << filename << std::endl;
//    }
//
//    return 0;
//}