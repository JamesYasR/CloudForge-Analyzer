#pragma once
// ============================================================
// MeasureWeldPreparation —— 焊前装配阶差(1.3) 与 焊前装配间隙(1.4)
//
// 依据: docs/当前需新增功能/焊前装配阶差与间隙-独立执行方案.md
//
// 设计要点(与方案逐条对应):
//  §1.1/§2.2 阶差 = 近件接缝边缘点与远件接缝边缘曲线在**同一轴向截面**上的
//            周向弧长距离 D = |Δs| = R·|Δφ|; 明确不是径向高低差,
//            近件径向抬高 1 mm 不改变阶差结果。
//  §1.2/§2.3 间隙方向是显式输入, 支持"沿圆柱轴向"与"曲面内接缝法向"两种定义,
//            结果必须带方向标签; 方向模式未定时不给出无方向说明的间隙结论。
//  §1.3/§3   先取两个试样, 再用各自 Z 均值做近远命名(可用人工覆盖);
//            Z 均值只用于命名, 不承担试样分割。
//  §1.4/§4.2 参考圆柱只用远件的有效壁面拟合, 半径固定为用户提供的设计半径;
//            不对两件合在一起拟合, 不启用焊后凸起焊缝约束
//            (即只调 MeasureCylindricity::evaluateCylindricity,
//             不调 evaluateCylindricityWithWeld)。
//  §1.5/§4.4 远件实际接缝边缘必须有观测支持; 无限延伸的拟合曲面不推断被遮住的板边。
//  §1.6/§4.4 栅格只做候选搜索与加速, 最终尺寸回到**有效原始边缘点**与受观测
//            支持的边缘折线计算。
//  §1.7/§4.5 样本、方向、边缘支持或轴线质量不足时返回明确的不可测原因;
//            缺测不是零间隙。
//  §4.6      主结果是有效原始边缘点对应距离的最大值; 被排除的候选极值附带理由
//            保留为诊断; 存在缺测区间时不宣称已确定整条缝的全局最大值。
//
// 本类为纯计算: 不依赖 Qt, 不构造查看器/VTK actor, 不使用 std::cout;
// 进度经 ProgressCallback 回报, 供 GUI 侧(worker 线程)转 PostProgress/PostLog。
// ============================================================

#include "config/pcl114.h"
#include "Basic/CylinderSurfaceFrame.h"

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class MeasureWeldPreparation
{
public:
    // 进度回调: (当前, 总量, 阶段描述); 返回 false 表示用户请求取消
    using ProgressCallback = std::function<bool(int, int, const std::string&)>;

    // ---- 测量指标(2026-09 口径统一) ----
    //   阶差 = 径向(e_P - e_Q, 近件在外为正); 间隙 = 沿跨缝方向(环缝=轴向)的开口宽度。
    //   两个量由**同一次配对**同时得到, 因此默认一起输出(Both)。
    enum class Metric {
        Step,   // 1.3 焊前装配阶差(径向)
        Gap,    // 1.4 焊前装配间隙(沿显式指定的跨缝方向)
        Both    // 合并入口: 一次配对同时给出阶差与间隙
    };

    // ---- 跨缝方向(间隙宽度方向; 也决定配对方向) ----
    enum class GapDirection {
        Axial,      // 沿圆柱轴向 q=(1,0): 等周向坐标寻找远边缘交点 —— 环缝用这个
        SeamNormal  // 曲面内接缝法向: 由两条接缝边缘的中心走向定切向, q 与 t 垂直并朝向远件
    };

    // ---- 结果状态(§1.7) ----
    enum class Status {
        Valid,        // 全程有效
        Partial,      // 部分有效: 存在缺测区间, 结论限定在有效测量段内
        Unmeasurable, // 不可测: 有明确原因(轴线不可靠 / 边缘无观测支持 / 方向退化 ...)
        Cancelled,    // 用户取消
        Failed        // 内部错误(参数非法 / 输入为空)
    };

    // ---- 试样角色 ----
    enum class Role { Near, Far, Undetermined };

    // ---- 参数 ----
    struct Params {
        double design_radius = 0.0;      // 设计半径 R(mm); 必须 > 0, 由用户提供
        Metric metric = Metric::Step;
        GapDirection gap_direction = GapDirection::Axial;

        double grid_cell = 0.0;          // 展开域栅格边长(mm); <=0 自动(按点距估计)
        double edge_neighborhood = 0.0;  // 边缘邻域半径(mm); <=0 自动(=3×grid_cell)
        // 边缘判据用"邻域角度空缺"(方案 §4.4 两级方法):
        //   栅格只负责挑出边界候选格; 真正的判据在**原始展开点**上按点距尺度的
        //   邻域角度空缺计算。这样边缘点位置直接落在真实采样点上, 误差是亚点距级,
        //   而不是一个栅格边长(否则 2mm 的阶差会被 3mm 的栅格直接吞掉)。
        double boundary_gap_deg = 100.0; // 最大角度空缺超过此值判为边界点
        double radius_min_factor = 0.50; // 邻域内半径 = factor × 点距(用于剔除自身)
        double radius_max_factor = 2.50; // 邻域外半径 = factor × 点距
        int    edge_directions = 16;     // 诊断用的方向数(记录空缺方向)
        // 支持比例 = 1 - 最大空缺/2π。一条干净的直边缘恰好是 0.50
        // (空缺 180°); 角点约 0.25; 完整内部表面 ≥ 0.85。默认 0.35。
        double min_edge_support = 0.35;  // 支持比例下限
        double facing_tol_deg = 60.0;    // 边缘朝向与"朝向对方试样"的允许夹角(度)
        double seam_search_max = 0.0;    // 与对方试样的最大允许间距(mm); <=0 自动(=10×grid_cell)
        int    min_edge_points = 6;      // 一条边缘折线段的最少原始点数
        double max_gap = 0.0;            // 折线断开阈值(mm); <=0 自动(=8×grid_cell)
        double search_window = 0.0;      // 对应点搜索窗口(mm); <=0 自动(=10×grid_cell)
        double edge_smooth_window = 0.0; // 远边局部稳健直线细化窗口(mm); <=0 自动(=6×grid_cell)
        double seam_exclude = 0.0;       // 远件拟合时排除接缝附近的带宽(mm); <=0 自动(=20×…见实现)
        double max_axis_stability_deg = 0.10;  // 子区拟合轴向最大允许夹角(度)
        double max_axis_stability_mm = 1.00;   // 子区拟合轴线位置最大允许偏移(mm)
        // 交点平行退化判据(§4.5 "平行判据依据交点误差的放大程度选择"):
        //   ratio    = |q × d| / |d|   (q=射线方向, d=远边折线段方向)
        //   amp      = sigma_seg / ratio  (交点沿射线方向的误差放大估计, mm)
        // ratio < parallel_ratio_min (硬退化) 或 amp > max_intersection_error 时拒测。
        double parallel_ratio_min = 0.02;      // ≈1.15°, 低于此值视为方向与边缘近乎平行
        double max_intersection_error = 0.0;   // 交点定位误差上限(mm); <=0 自动(=2×grid_cell)
        double platform_tol = 0.0;             // 平台区判定容差(mm); <=0 自动(=max(2×grid_cell,0.01))
        bool   verbose = false;
    };

    // ---- 展开后的一个边缘点(保留原始索引与三维坐标) ----
    struct EdgePoint {
        int    source_index = -1;                 // 原始点云索引(可逆追溯)
        bool   from_near = true;                  // true=近件, false=远件
        double a = 0.0, s = 0.0;                  // 展开坐标(mm)
        double e = 0.0;                           // 到理想柱面的径向偏差(mm)
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        double support = 0.0;                     // 本侧表面支持比例 [0,1]
        double outward_deg = 0.0;                 // 空缺方向(度, 展开面内, 相对 +a 轴)
        double to_other_mm = 0.0;                 // 到对方试样最近点的展开面距离(mm)
        bool   touch_roi = false;                 // 是否贴着测量区域边界(人工裁剪边界)
        bool   valid = false;                     // 是否通过"接缝侧有效边缘"筛选
        std::string reject_reason;                // 未通过时的原因
    };

    // ---- 一条边缘(分段折线; 断点处不连线) ----
    struct Edge {
        std::vector<EdgePoint> points;            // 全部候选(含被拒绝的, 便于诊断)
        std::vector<std::vector<int>> polylines;  // 按走势排序后的有效点索引分段
        std::vector<std::vector<Eigen::Vector2d>> refined; // 每段的局部稳健细化折线 (a,s)
        std::vector<double> refined_sigma;        // 每段细化折线的局部拟合残差 rms(mm)
        int    candidate_count = 0;
        int    valid_count = 0;
        double a_min = 0, a_max = 0, s_min = 0, s_max = 0;
        bool   ok = false;
        std::string note;
    };

    // ---- 一个近件边缘点的配对结果 ----
    struct Match {
        int    near_index = -1;                   // 近件原始点索引
        bool   valid = false;
        std::string invalid_reason;
        double near_a = 0, near_s = 0;
        double near_e = 0;                        // 近件边缘点的径向残差 e_P = rho - R
        Eigen::Vector3d near_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d near_proj_xyz = Eigen::Vector3d::Zero(); // 径向投影 P'
        int    far_seg = -1;                      // 命中的远边折线段号
        int    far_point_index = -1;              // 命中的远边原始点索引(最近原始点)
        double far_a = 0, far_s = 0;
        double far_e = 0;                         // 命中远边点的径向残差 e_Q
        Eigen::Vector3d far_xyz = Eigen::Vector3d::Zero();
        double lambda = 0.0;                      // 间隙: 沿跨缝方向 q 的有符号 λ
        double distance = 0.0;                    // 间隙: |λ|
        // 径向阶差(与间隙同一次配对得到; 口径: 近件在外为正)
        double step_signed = 0.0;                 // Δe = e_P - e_Q
        double step_abs = 0.0;                    // |Δe|
        bool   widened = false;                   // 该点是靠"扩窗重试"才配上的(需人工确认)
    };

    // ---- 最大值项(间隙与阶差各一份) ----
    struct MaxItem {
        bool   ok = false;
        double value = 0.0;                       // 非负主结果(间隙 |λ| 或阶差 |Δe|)
        double lambda_signed = 0.0;               // 对应的有符号量(λ 或 Δe)
        int    near_index = -1;
        double near_a = 0, near_s = 0;
        Eigen::Vector3d near_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d near_proj_xyz = Eigen::Vector3d::Zero();
        double far_a = 0, far_s = 0;
        Eigen::Vector3d far_xyz = Eigen::Vector3d::Zero();
        std::vector<int> ties;                    // 并列/平台区对应的近件点索引
        bool   needs_review = false;              // 极值邻域支持不足, 结论待复核
        std::string review_reason;
    };

    // ---- 诊断: 被排除的候选极值 ----
    struct ExcludedCandidate {
        double distance = 0.0;
        int    near_index = -1;
        std::string reason;
    };

    // ---- 完整结果 ----
    struct Result {
        Status status = Status::Failed;
        std::string reason;                       // 不可测/失败原因(客户可见中文)
        std::string report;                       // 报告文本(结论先行)

        // 输入与角色(§3)
        std::string near_cloud_name, far_cloud_name;
        Role   near_role = Role::Near;
        Role   far_role = Role::Far;
        bool   role_overridden = false;           // 是否人工交换了近远
        std::string role_note;
        double near_z_mean = 0.0, far_z_mean = 0.0;
        int    near_points = 0, far_points = 0;
        bool   single_cloud_roi = false;          // 是否单点云内存 ROI 模式

        // 参考圆柱基准(§4.2)
        bool   fit_ok = false;
        Eigen::Vector3d axis_point = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis_direction = Eigen::Vector3d::UnitZ();
        double design_radius = 0.0;
        double fit_rms = 0.0;                     // 远件拟合子集上的径向 RMS(mm)
        double fit_mean = 0.0;
        double fit_p2p = 0.0;
        double fit_coverage_a = 0.0;              // 拟合点在轴向的覆盖长度(mm)
        int    fit_points = 0;
        int    fit_iterations = 0;                // 稳健剔除轮数
        double axis_stability_deg = 0.0;          // 子区拟合轴向最大夹角(度)
        double axis_stability_mm = 0.0;           // 子区拟合轴线位置最大偏移(mm)
        bool   baseline_reliable = false;
        std::string baseline_note;
        bool   axis_prior_used = false;

        // 参数与实际取值(便于追溯)
        Metric metric = Metric::Step;
        GapDirection gap_direction = GapDirection::Axial;
        std::string direction_label;              // 方向标签(必须进入报告)
        double grid_cell = 0, edge_neighborhood = 0, search_window = 0;
        double actual_point_pitch = 0.0;          // 估计的点距(mm)
        double roi_a0 = 0, roi_a1 = 0, roi_s0 = 0, roi_s1 = 0;

        // 边缘(§4.4)
        Edge near_edge, far_edge;

        // 配对(§4.5)
        std::vector<Match> matches;
        int    valid_count = 0;
        int    invalid_count = 0;
        int    widen_retry_hits = 0;              // 靠"扩窗重试"才配上的点数(诊断用, 不打印)
        double valid_length = 0.0;                // 有效测量段长度(沿接缝走向, mm)
        std::vector<std::string> missing_intervals;   // 缺测区间描述
        std::vector<std::string> invalid_reasons;     // 逐类无效原因统计

        // 最大值(§4.6): 间隙与径向阶差各一份, 均来自同一批配对点
        MaxItem maximum;                          // 间隙(沿跨缝方向)最大值
        MaxItem maximum_step;                     // 径向阶差最大值
        double gap_median = 0.0;                  // 间隙: 全部有效点的整体中值
        double step_median = 0.0;                 // 径向阶差: 全部有效点的整体中值
        bool   has_platform = false;              // 间隙最大位置存在平台区
        double platform_a0 = 0, platform_a1 = 0, platform_s0 = 0, platform_s1 = 0;
        bool   has_platform_step = false;         // 阶差最大位置存在平台区
        double platform_step_a0 = 0, platform_step_a1 = 0;
        double platform_step_s0 = 0, platform_step_s1 = 0;
        std::vector<ExcludedCandidate> excluded;      // 间隙的并列/存疑候选
        std::vector<ExcludedCandidate> excluded_step; // 阶差的并列/支撑不足候选
        int    step_unsupported = 0;              // 其中"邻域支撑不足"(疑似孤立尖刺/坏点)的个数
        bool   is_global_max = false;             // 是否可称为整条缝的全局最大值

        void reset() { *this = Result(); }
    };

    MeasureWeldPreparation();
    ~MeasureWeldPreparation();

    // ---------- 输入 ----------
    // 方式一(推荐, 优先打通): 两个已有点云。
    void setNearCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name);
    void setFarCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name);
    // 方式二: 单份原始点云 + 人工圈选的内存 ROI 索引(保留源索引, 不落临时文件)。
    void setSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& name);
    void setNearIndices(std::vector<int> indices);
    void setFarIndices(std::vector<int> indices);

    // ---------- 参数 ----------
    void setParams(const Params& p) { params_ = p; }
    const Params& params() const { return params_; }

    // 测量区域(展开域 (a,s) 矩形)。必须在 evaluate() 之前给出:
    // 展开坐标依赖参考圆柱基准, 因此这是"选定接缝区域/有效测量段"的接口。
    // 若不给(a 全为 0 且 s 全为 0), 则自动取两件投影点的公共包围盒。
    void setSeamRoi(double a0, double a1, double s0, double s1);

    // 扫描坐标约定(§3): Z 增大是否代表远离扫描仪。
    // convention_known=false 表示现场没有该约定(坐标已变换/未知), 此时必须
    // 人工指定近远角色, 否则返回"角色待指定"而不是悄悄按 Z 均值命名。
    void setZConvention(bool convention_known, bool increase_means_away)
    {
        z_convention_known_ = convention_known;
        z_increase_away_ = increase_means_away;
    }
    // 人工指定近远角色(覆盖按 Z 均值的自动判定), 并记录原因(§3)。
    void setRoleOverride(Role near, Role far, const std::string& reason);
    void clearRoleOverride();

    // 参考圆柱轴线先验(可选; 用于"必要时要求外部轴向先验", §4.2)。
    // 语义是"初始值": 仍会对远件壁面做固定半径优化, 并做子区稳定性检查。
    void setAxisPrior(const Eigen::Vector3d& point, const Eigen::Vector3d& direction);

    // 外部权威轴线: 直接采用给定 O/u(半径仍用 design_radius), 不做远件拟合、
    // 不做子区稳定性检查。用于 A/B 层验证(真值轴线), 或现场已有可靠设计轴的情形。
    // 报告会明确写出"使用外部给定轴线", 不会与自拟合结果混淆。
    void setFixedReferenceAxis(const Eigen::Vector3d& point, const Eigen::Vector3d& direction);

    // ---------- 执行 ----------
    void setProgressCallback(ProgressCallback cb) { progress_cb_ = std::move(cb); }
    bool isCancelled() const { return cancelled_; }
    void setCancelled(bool c) { cancelled_ = c; }

    Result evaluate();

    // 供 GUI 侧在 evaluate() 后取用(与 Result 内容一致)
    const CylinderSurfaceFrame& frame() const { return frame_; }

private:
    // ---- 流程步骤 ----
    bool checkInputs(std::string& reason) const;                       // 参数/输入有限性与合法性
    bool resolveRoles(Result& r, std::string& reason);                 // §3 近远判定
    bool buildReferenceCylinder(Result& r, std::string& reason);       // §4.2 远件固定半径基准 + 稳定性
    bool buildFrameAndProject(Result& r, std::string& reason);         // §2.1 展开两件
    bool buildEdges(Result& r, std::string& reason);                   // §4.4 接缝侧真实边缘
    bool buildEdgeFor(bool from_near, Edge& out, const std::string& tag); // 单侧边缘
    bool pairAlongDirection(Result& r, std::string& reason);           // §4.5 沿跨缝方向配对
    void findMaximumAndDiagnostics(Result& r);                         // §4.6 最大值/平台/排除候选
    // 单个量(use_step=false 取间隙 |λ|, true 取阶差 |Δe|)的最大值/平台/并列候选
    void findMaximumFor(Result& r, bool use_step, MaxItem& mx, double& median_out,
                        bool& has_platform,
                        double& pa0, double& pa1, double& ps0, double& ps1,
                        std::vector<ExcludedCandidate>& excluded) const;
    void composeReport(Result& r) const;                               // §7 报告文本

    // ---- 工具 ----
    bool reportProgress(int current, int total, const std::string& stage);
    double estimatePointPitch(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                              const std::vector<int>& indices) const;
    // 固定半径下的轴线局部精化(坐标下降): 以 rms 为目标, 只在 (c,u) 邻域内搜索。
    // 存在原因是 evaluateCylindricity() 的 400 方向全局搜索代价高, 不能用于
    // 逐子区稳定性检查(§4.2); 这里只做盆地内的抛光。
    double refineAxis(const std::vector<Eigen::Vector3d>& pts,
                      Eigen::Vector3d& c, Eigen::Vector3d& u,
                      double cStep, double uStepDeg, int rounds) const;
    double axisRms(const std::vector<Eigen::Vector3d>& pts,
                   const Eigen::Vector3d& c, const Eigen::Vector3d& u) const;
    // 展开面射线与折线段求交(返回 0=无交点, 1=唯一, >1=多交点)
    struct RayHit {
        double lambda = 0.0;
        int    seg = -1;                 // 命中的折线段号
        double px = 0.0, py = 0.0;       // (a,s) 交点
        double ratio = 1.0;              // |q × d| / |d|, 越小越平行(退化)
        double amp = 0.0;                // 交点沿射线方向误差放大估计 = sigma/ratio (mm)
        double sigma = 0.0;              // 该段细化折线的局部拟合残差(mm)
        bool   rejected = false;         // 是否因平行退化/放大超限被剔除
    };
    // 求交并同时做平行退化筛选。bothSides=false 时只取 lambda>=0 一侧。
    // 返回"未被剔除"的交点簇数(按 lambda 聚类); hits 内含全部(含 rejected)命中。
    int intersectRayWithPolylines(const Eigen::Vector2d& origin,
                                  const Eigen::Vector2d& q,
                                  const std::vector<std::vector<Eigen::Vector2d>>& polylines,
                                  const std::vector<double>& sigmas,
                                  double maxLambda,
                                  bool bothSides,
                                  double parallel_ratio_min,
                                  double max_amp,
                                  std::vector<RayHit>& hits,
                                  int& parallel_rejects,
                                  int& amp_rejects) const;
    // 找到展开面上离 (a,s) 最近的对方试样原始点(用于"朝向对方"判据与 seam_search_max)
    int nearestOtherPoint(bool other_is_near, double a, double s, double& dist) const;
    // 同侧原始展开点的邻域角度空缺判据(方案 §4.4): 返回 false 表示邻域点太少无法判定。
    // support = 1 - 最大空缺/2π; outward_rad = 最大空缺的角平分线方向(展开面内, 相对 +a 轴)。
    bool supportByAngularGap(bool from_near, double a, double s,
                             double& support, double& outward_rad, int& neighbor_count) const;
    // 从 ROI 内的 (a,s) KD 树估计真实点距: 随机抽样查询"第 2 近邻"取中位数。
    // 不能用"等步长抽样子集内部找最近邻": 那样得到的是抽样间距, 会把点距高估十几倍,
    // 进而把角度空缺的邻域半径放大到把整条边缘带都误判成边界。
    double estimatePitchFromKd(const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kd,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& holder) const;
    // 同一侧的多段边缘是否只是"同一条缝的断点"(互相共线且垂距接近)。
    // 不共线 => 区域内有多条接缝候选, 第一版要求先缩小 ROI(§4.4)。
    bool edgeSegmentsCompatible(const Edge& e, std::string& why) const;

    // 简单均匀栅格(展开域), 只做候选搜索与加速(§1.6)
    struct Grid {
        int na = 0, ns = 0;
        double a0 = 0, s0 = 0, cell = 1.0;
        std::vector<char> occupied;
        std::vector<std::vector<int>> buckets;   // 每格内的点索引(局部索引)
        bool inside(int ia, int is) const {
            return ia >= 0 && is >= 0 && ia < na && is < ns;
        }
        int idx(int ia, int is) const { return ia * ns + is; }
        void cellOf(double a, double s, int& ia, int& is) const {
            ia = static_cast<int>(std::floor((a - a0) / cell));
            is = static_cast<int>(std::floor((s - s0) / cell));
        }
        bool occ(int ia, int is) const {
            return inside(ia, is) && occupied[idx(ia, is)] != 0;
        }
    };

    // ---- 输入 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud_, far_cloud_, single_cloud_;
    std::string near_name_, far_name_, single_name_;
    std::vector<int> near_indices_, far_indices_;
    bool single_mode_ = false;

    Params params_;
    bool   roi_set_ = false;
    double roi_a0_ = 0, roi_a1_ = 0, roi_s0_ = 0, roi_s1_ = 0;
    bool   z_convention_known_ = true;
    bool   z_increase_away_ = true;
    bool   role_override_ = false;
    Role   role_near_ = Role::Near, role_far_ = Role::Far;
    std::string role_override_reason_;
    bool   axis_prior_set_ = false;
    Eigen::Vector3d prior_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prior_dir_ = Eigen::Vector3d::UnitZ();
    bool   axis_fixed_ = false;
    Eigen::Vector3d fixed_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d fixed_dir_ = Eigen::Vector3d::UnitZ();

    ProgressCallback progress_cb_;
    bool cancelled_ = false;

    // ---- 中间状态 ----
    CylinderSurfaceFrame frame_;
    // 全量投影结果只保留展开标量 + 原始索引(原始三维坐标按需由源点云取回),
    // 避免在百万点级输入上再复制一份 Vector3d 数组。
    std::vector<int>    near_src_, far_src_;   // 原始点云索引
    std::vector<double> near_a_, near_s_, near_e_;
    std::vector<double> far_a_,  far_s_,  far_e_;
    Grid near_grid_, far_grid_;
    // (a,s,0) 形式的点云 + KD 树: 只用于"朝向对方试样"与 seam_search_max 判据。
    // 只装入落在测量区域(ROI)内的点, 避免在百万点级输入上复制整份点云。
    pcl::PointCloud<pcl::PointXYZ>::Ptr as_near_holder_, as_far_holder_;
    std::vector<int> near_kd_map_, far_kd_map_;   // KD 索引 -> 展开数组局部索引
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr near_kd_, far_kd_;
    double point_pitch_ = 1.0;
    double fit_rms_ = 0.0, fit_mean_ = 0.0, fit_p2p_ = 0.0;
    double axis_stab_deg_ = 0.0, axis_stab_mm_ = 0.0;
    std::vector<int> fit_subset_indices_;   // 远件拟合子集(远件局部索引)
    std::string baseline_note_;

    int stage_index_ = 0;
    static constexpr int kStageTotal = 8;
};
