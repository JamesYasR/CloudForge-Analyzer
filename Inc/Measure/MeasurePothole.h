#pragma once
#include "config/pcl114.h"
#include <memory>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <Eigen/Dense>

// ============================================================
// 凹塘/凹坑测量
// 调用方式与圆柱度测量一致: 用户先通过"拟合圆柱(二次/三次优化)"得到
// 圆柱结果并自动存入 cylinderResultsMap, 本类直接使用选定的
// 轴线 + 理想半径(即优化时输入的设计半径), 不再内部重新拟合.
// 计算流程: 全量点到理想柱面的带符号径向距离(凹为负) -> 标注最深点;
//       按距离阈值提取凹塘点群(欧氏聚类取最大簇); 在展开面
//       (轴向 x 周向弧长)内提取点群轮廓并做最小二乘椭圆拟合,
//       得到凹塘长短轴, 椭圆回投柱面供可视化.
//
// 局部基准(口径B, docs/当前需新增功能/凹塘测量-局部基准架构设计.md §3):
//   口径A e = rho - R 只在"形面偏差 << 坑深"时才能作为判据; 真实壁面有 mm 级
//   碗形/倾斜形面偏差时会把整片偏差判成凹塘. 因此:
//     展开域(a,s)栅格 + 每格中值 m  ->  半径 W/2 窗口内截尾最小二乘二阶拟合
//     得到局部基准 b(a,s)  ->  局部凹陷 d = b - m (正 = 比周围低)
//     判定用 d > T_local; 距补丁边界 < W/2 的格不参与判定.
//   口径A 的数值(最大距离/最深点/sigma)始终照旧报告, 不受口径B 影响.
// ============================================================
class MeasurePothole
{
public:
    // ---- 单个凹坑(多凹坑逐个报告用) ----
    // 多凹坑需求: "提取偏离柱面的点群 -> 轮廓 -> 椭圆 -> 长短轴", 每个满足
    // min_cluster_size 的簇都要有一份独立结果(四个坑 = 四个结果).
    struct PitItem {
        int    index = 0;               // 编号(1 起, 按点群点数降序: 1 = 主坑)
        bool   is_main = false;         // 是否为主坑(面积最大者, 同时保留在 PitResult 原有字段里)
        int    pit_points = 0;          // 该簇点数
        int    pit_cells = 0;           // 该簇占据的判定格数
        // 深度: 局部口径B(相对局部基准的下凹, 正=凹)
        double local_max_depth = 0.0;   // 该坑局部最大下凹(mm)
        double local_mean_depth = 0.0;  // 该坑点群平均局部下凹(mm)
        // 深度: 口径A(到理想柱面, 正=凹深) —— 兼容/对照
        double global_max_depth = 0.0;  // 点群内最大 -e
        double global_mean_depth = 0.0; // 点群平均 -e
        // 几何(展开域 a=轴向, s=周向弧长, 单位 mm)
        double major_axis = 0.0;        // 椭圆长轴(2a, 已做阈值掩膜内缩补偿)
        double minor_axis = 0.0;        // 椭圆短轴(2b, 已做阈值掩膜内缩补偿)
        double major_axis_raw = 0.0;    // 未补偿的原始拟合长轴(口径原始值, 供复核)
        double minor_axis_raw = 0.0;    // 未补偿的原始拟合短轴
        double aspect_ratio = 0.0;      // 长短轴比
        double ellipse_center_a = 0.0;  // 椭圆中心(展开域)
        double ellipse_center_s = 0.0;
        double contour_span_a = 0.0;    // 点群展开外接盒(轴向/周向跨度, 用于比对)
        double contour_span_s = 0.0;
        double centroid_a = 0.0;        // 点群质心(展开域)
        double centroid_s = 0.0;
        pcl::PointXYZ centroid;         // 点群质心(3D)
        pcl::PointXYZ deepest_point;    // 该坑最深点(局部口径, 无局部基准时用口径A)
        // 校验(四道闸门)
        bool   ellipse_ok = false;      // 轮廓/椭圆拟合是否成功
        bool   valid = false;           // 四道闸门是否全部通过
        std::string reject_reason;      // 拒绝原因(valid=false 时)
        double area_fraction = 0.0;     // 点群面积占补丁比例
        bool   touch_boundary = false;  // 是否贴补丁边界
        bool   ellipse_out_of_patch = false; // 椭圆是否越出补丁
        bool   judged_by_local = false; // 该坑是否以局部口径判定
        std::vector<Eigen::Vector3f> ellipse_points; // 椭圆回投柱面采样点(闭合折线)
    };

    struct PitResult {
        bool fit_ok;                    // 圆柱参数是否有效
        bool valid;                     // 是否提取到凹塘点群并完成椭圆拟合
        double max_depth;               // 最大凹深(mm, 正值)
        pcl::PointXYZ deepest_point;    // 最深点坐标
        double mean_depth;              // 点群平均深度(mm, 正值)
        double major_axis;              // 椭圆长轴(mm)
        double minor_axis;              // 椭圆短轴(mm)
        int pit_points;                 // 凹塘点群点数
        int cluster_count;              // 检出的偏离点群簇数
        double robust_sigma;            // 残差稳健 sigma(mm)
        double design_radius;           // 理想半径(mm)
        // 形面趋势(系统性偏差)诊断
        bool trend_removed = false;     // 提点群时是否扣除了形面趋势
        double trend_p2p = 0.0;         // 形面趋势峰峰值(mm)
        double trend_ratio = 0.0;       // 趋势/局部起伏 比值
        double pit_area_fraction = 0.0; // 点群面积占补丁比例
        bool pit_touch_boundary = false;// 点群是否贴补丁边界
        Eigen::Vector3f axis_point;     // 拟合轴上一点
        Eigen::Vector3f axis_direction; // 拟合轴向(单位向量)
        std::string assessment_message; // 评估结果消息

        // ---- 多凹坑: 逐个报告(主坑 = 面积最大者, 同时保留在上面原有字段里) ----
        std::vector<PitItem> pits;      // 全部检出凹坑(按点数降序), 空 = 未检出
        int pit_count = 0;              // 检出凹坑数(= pits.size())
        double grid_cell = 0.0;         // 本次局部基准实际使用的格边长(mm), 便于追溯
        double local_window = 0.0;      // 本次实际使用的窗口 W(mm)

        // ---- 局部基准(口径B)结果, 详见规格 §5 ----
        double local_max_depth = 0.0;          // 局部最大下凹(mm, 正=比周围低)
        double local_threshold = 0.0;          // 实际采用的局部阈值(mm)
        double baseline_offset_p2p = 0.0;      // 局部基准相对理想柱面的偏移峰峰值(=形面偏差量, mm)
        double boundary_excluded_fraction = 0.0; // 被排除的边界区面积占比
        bool   judged_by_local = false;        // 本次是否以局部口径判定
        double local_mean_depth = 0.0;         // 检出点群的平均局部下凹(mm)
        pcl::PointXYZ local_deepest_point;     // 局部口径最深点(与口径A 的 deepest_point 分开记录)

        PitResult() : fit_ok(false), valid(false), max_depth(0.0), deepest_point(),
            mean_depth(0.0), major_axis(0.0), minor_axis(0.0), pit_points(0),
            cluster_count(0), robust_sigma(0.0), design_radius(0.0),
            axis_point(Eigen::Vector3f::Zero()), axis_direction(Eigen::Vector3f::UnitZ())
        {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            local_deepest_point = pcl::PointXYZ(nan, nan, nan);
        }
    };

    MeasurePothole();
    ~MeasurePothole();

    // 设置输入参数
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setCylinder(const pcl::ModelCoefficients::Ptr& cyl); // 已保存的圆柱拟合结果(7值)
    void setDistanceThreshold(double thr);    // 距离阈值(mm), <=0 自动(稳健阈值)
    void setClusterTolerance(double tol);     // 聚类容差(mm), <=0 自动(按点距估计)
    void setMinClusterSize(int n);            // 点群最小点数
    void setVerbose(bool verbose);

    // 形面趋势(系统性偏差)处理: 真实壁面相对理想圆柱总有 mm 级平滑形面偏差,
    // 若其幅度可与阈值相比, 低于阈值的点会连成大片"假凹塘".
    // 模式: 0=自动(趋势显著时扣除), 1=不扣除, 2=强制扣除
    void setTrendMode(int mode) { trend_mode_ = mode; }
    void setTrendOrder(int order) { trend_order_ = std::max(1, std::min(2, order)); }
    // 点群合理性上限(校验不通过则不判定为凹塘)
    void setMaxAreaFraction(double f) { max_area_fraction_ = (f > 0 ? f : 0.20); }
    void setMaxAspectRatio(double r) { max_aspect_ratio_ = (r > 0 ? r : 5.0); }

    // ---- 局部基准(口径B)参数, 规格 §5 ----
    // 默认 true: 用"相对局部基准的下凹量 d > T_local"作为凹塘判据;
    // 置 false 则回到旧的"到理想柱面距离 e < -阈值"判据(数值与改造前一致).
    void setUseLocalBaseline(bool on) { use_local_baseline_ = on; }
    void setLocalWindow(double mm) { if (mm > 0.0) local_window_ = mm; }   // 局部基准窗口 W(mm), 默认 80
    void setLocalThreshold(double mm) { if (mm >= 0.0) local_threshold_ = mm; } // 局部阈值(mm), 默认 0.5
    void setBoundaryExclude(bool on) { boundary_exclude_ = on; }           // 距边界 < W/2 的格不参与判定
    // 热力图显示哪个场: 0=局部凹陷深度 d(默认), 1=到理想柱面距离 e(口径A)
    void setHeatMapField(int field) { heatmap_field_ = (field == 1 ? 1 : 0); }

    // 执行测量
    PitResult evaluate();

    // 可视化数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPitCloud() const { return pit_cloud_; }            // 凹塘点群
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getHeatMapCloud() const { return heatmap_cloud_; } // 残差热力图
    // 实际显示的热力图场: 0=局部凹陷 d(口径B), 1=到理想柱面距离 e(口径A)
    int getHeatMapFieldUsed() const { return heatmap_field_used_; }
    const std::vector<Eigen::Vector3f>& getEllipsePoints() const { return ellipse_points_; }  // 主坑椭圆回投3D采样点(闭合折线)
    // 多凹坑: 逐个标注用. 每项 index 与 PitResult::pits 一一对应(1 起, 降序);
    // ellipse_points 为空表示该坑椭圆拟合失败或校验未通过(不绘制).
    const std::vector<PitItem>& getPits() const { return pit_items_; }
    double getGridCell() const { return lb_cell_; }
    double getLocalWindow() const { return local_window_; }

private:
    // 步骤实现
    bool computeResiduals();          // 全量点带符号径向残差(相对选定圆柱)
    void estimateFormTrend();         // 展开域低阶趋势(形面偏差)估计与扣除
    bool computeLocalBaseline();      // 展开域栅格 + 大窗口稳健二阶拟合 -> 局部基准与局部凹陷 d
    bool extractPitCluster();         // 阈值+聚类提取全部凹塘点群(多簇不丢弃)
    bool fitEllipseOnContour();       // 展开域轮廓提取+椭圆拟合+回投(对 pit_cloud_ 当前内容)
    bool validatePit(std::string& reason); // 点群合理性校验(贴边/面积占比/长宽比/椭圆越界)
    bool buildPitItem(int index, const pcl::PointIndices& cluster, const std::vector<int>& candToOrig,
                      bool useLocal, PitItem& out); // 单簇 -> PitItem(点群/椭圆/校验/质心全流程)

    // 工具函数
    void generateHeatMapCloud();      // 残差热力图(蓝凹红凸), 场的选取见 heatmap_field_
    void printDebugInfo(const std::string& message) const;
    bool isPointValid(const pcl::PointXYZ& point) const;

    // 输入
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    pcl::ModelCoefficients::Ptr cylinder_;   // 选定的圆柱结果
    double distance_threshold_;              // 距离阈值(mm), <=0 自动
    double cluster_tolerance_;               // 聚类容差(mm), <=0 自动
    int min_cluster_size_;                   // 最小点群点数
    bool verbose_;

    // 中间结果
    std::vector<double> residual_map_;   // 每点带符号径向残差 e = rho - R (原始, 需求口径)
    std::vector<double> residual_work_;  // 用于缺陷提取的残差(可能已扣除形面趋势)
    double robust_sigma_;                // 原始残差稳健 sigma
    double robust_sigma_work_;           // 工作残差稳健 sigma
    double effective_threshold_;         // 实际采用的距离阈值
    bool trend_removed_ = false;         // 本次是否扣除了形面趋势
    double trend_p2p_ = 0.0;             // 形面趋势峰峰值(mm)
    double trend_ratio_ = 0.0;           // 趋势/局部起伏 比值
    double patch_area_ = 0.0;            // 补丁面积估计(mm^2)
    double pit_area_fraction_ = 0.0;     // 点群面积占补丁比例
    bool pit_touch_boundary_ = false;    // 点群是否贴补丁边界
    double ellipse_aspect_ = 0.0;        // 椭圆长短轴比
    bool ellipse_out_of_patch_ = false;  // 椭圆是否越出补丁
    // 形面趋势与校验参数
    int trend_mode_ = 0;                 // 0=自动 1=不扣除 2=强制扣除
    int trend_order_ = 2;                // 1=平面 2=二次
    double max_area_fraction_ = 0.20;    // 点群面积占比上限
    double max_aspect_ratio_ = 5.0;      // 椭圆长短轴比上限
    // 补丁范围(展开域, 全量点)与椭圆几何(用于越界校验)
    double patch_a_min_ = 0.0, patch_a_max_ = 0.0;
    double patch_s_min_ = 0.0, patch_s_max_ = 0.0;
    double ellipse_cx_ = 0.0, ellipse_cy_ = 0.0, ellipse_angle_ = 0.0;
    Eigen::Vector3f axis_point_;         // 轴上一点
    Eigen::Vector3f axis_direction_;     // 轴向(单位向量)
    Eigen::Vector3f t1_, t2_;            // 垂直于轴的正交框架(展开用)
    double design_radius_;               // 理想半径(mm)

    // ---- 局部基准(口径B): 参数 ----
    bool use_local_baseline_ = true;     // 是否以局部口径判定
    double local_window_ = 90.0;         // 局部基准窗口 W(mm), 默认 90(≈3x 最大坑 30mm)
    double local_threshold_ = 0.35;      // 局部阈值 T_local(mm), 默认 0.35(见报告说明)
    bool boundary_exclude_ = true;       // 距补丁边界 < W/2 的格不参与判定
    int heatmap_field_ = 0;              // 热力图: 0=局部凹陷 d, 1=到理想柱面 e
    int heatmap_field_used_ = 1;         // 实际绘制的场(局部不可用时回退为 e)

    // ---- 局部基准(口径B): 结果 ----
    bool local_ok_ = false;              // 局部基准是否成功建立
    int lb_na_ = 0, lb_ns_ = 0;          // 栅格尺寸
    double lb_cell_ = 0.0;               // 栅格边长(mm)
    double lb_a0_ = 0.0, lb_s0_ = 0.0;   // 栅格原点(展开域 mm)
    double lb_stride_ = 1.0;             // 基准拟合节点间隔(格), >1 时节点间双线性插值
    std::vector<double> lb_med_;         // 每格点中值 m(NaN=空格)
    std::vector<double> lb_base_;        // 每格局部基准 b(NaN=未建立)
    std::vector<double> lb_dent_;        // 每格局部凹陷 d = b - m(NaN=未判定)
    std::vector<double> lb_dent_all_;    // 每格局部凹陷 d(不受边界排除影响, 供逐坑深度统计)
    std::vector<char> lb_judge_;         // 每格是否参与判定(1=参与)
    std::vector<int> lb_cell_of_;        // 每点所属格(-1=无效点)
    std::vector<double> local_dent_;     // 每点局部凹陷 d(NaN=未判定/不可用), 供热力图
    double local_max_depth_ = 0.0;       // 判定区内最大局部下凹(mm)
    double local_mean_depth_ = 0.0;      // 检出点群平均局部下凹(mm)
    double baseline_offset_p2p_ = 0.0;   // 局部基准相对理想柱面偏移峰峰值(mm)
    double boundary_excluded_fraction_ = 0.0; // 边界排除格占比
    pcl::PointXYZ local_deepest_point_ = pcl::PointXYZ(  // 局部口径最深点
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN());

    // 输出
    pcl::PointCloud<pcl::PointXYZ>::Ptr pit_cloud_;        // 主坑点群(面积最大者; 兼容原有单坑语义)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud_; // 热力图点云
    std::vector<Eigen::Vector3f> ellipse_points_;          // 主坑椭圆回投采样点
    std::vector<PitItem> pit_items_;                       // 全部检出凹坑(按点数降序)
    double ellipse_major_;                                 // 长轴(已补偿)
    double ellipse_minor_;                                 // 短轴(已补偿)
    double ellipse_major_raw_;                             // 长轴(未补偿)
    double ellipse_minor_raw_;                             // 短轴(未补偿)
    double pit_mean_depth_;                                // 点群平均深度
    int cluster_count_;                                    // 簇数
    int deepest_index_;                                    // 最深点在输入云中的索引
};
