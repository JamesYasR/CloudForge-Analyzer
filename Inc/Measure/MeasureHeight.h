#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <memory>
#include <vector>

class MeasureHeight {
public:
    // 构造：第一个为被测量点云，第二个为用于拟合参考平面的点云
    MeasureHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr measure_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud);

    // 执行拟合与测量，返回 true 表示成功
    bool measure();

    // 获取结果（单位与输入点云单位一致）
    double GetMaxDistance() const;   // 最大（绝对）距离
    double GetMinDistance() const;   // 最小（绝对）距离
    double GetMeanDistance() const;  // 平均（绝对）距离

    // 如果需要参考平面系数（a,b,c,d）
    pcl::ModelCoefficients::Ptr GetPlaneCoefficients() const;

private:
    bool fitPlane();               // 拟合参考平面，返回是否成功
    void computeDistances();       // 计算点到平面的距离统计

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr measure_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;
    pcl::ModelCoefficients::Ptr plane_coeffs_;

    bool measured_ = false;

    // 统计结果（基于绝对距离）
    double max_dist_ = 0.0;
    double min_dist_ = 0.0;
    double mean_dist_ = 0.0;

    // 内部存储的每点绝对距离（可选）
    std::vector<double> distances_;
};