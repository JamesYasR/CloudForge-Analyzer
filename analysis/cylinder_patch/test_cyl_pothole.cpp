// test_cyl_pothole.cpp -- 无界面验证: 圆柱定半径寻优加速 + 凹塘测量正确性
// 用法: ./test_cyl_pothole <pcd路径> [设计半径=1940] [选项...]
// 选项:
//   cancel / weld        保持原有语义(取消模拟 / 三阶段焊缝路径)
//   axis=<file>          轴线缓存: 文件存在则直接读取(跳过寻优), 否则寻优后写入
//                        -> 用于"改造前/改造后口径A 逐位一致"的对照
//   trend=<0|1|2>        形面趋势处理模式
//   area=<pct>           点群面积占比上限(%)
//   thr=<mm>             距离阈值(口径A)
//   local=<0|1>          是否使用局部基准(口径B)判定
//   win=<mm>             局部基准窗口 W
//   lthr=<mm>            局部阈值 T_local
//   bexcl=<0|1>          边界排除
//   heat=<0|1>           热力图显示 0=局部凹陷d 1=到理想柱面e
//   minpts=<n>           最小点群点数(默认200)
//   ctol=<mm>            聚类容差(默认0=自动3mm)
#include "Measure/MeasureCylindricity.h"
#include "Measure/MeasurePothole.h"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>

#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <pcd> [design_radius] [选项...]\n";
        return 1;
    }
    const double designR = (argc > 2) ? std::atof(argv[2]) : 1940.0;

    // ---- 选项解析 ----
    bool doCancel = false, useWeld = false;
    std::string axisFile;
    int trendMode = 0;
    double areaPct = 20.0, thr = 0.0, localWin = 90.0, localThr = 0.35;
    int useLocal = 1, bexcl = 1, heatField = 0;
    int minPts = 200;
    double cTol = 0.0;
    for (int i = 3; i < argc; ++i) {
        const std::string t = argv[i];
        auto val = [&](const char* key) { return t.substr(std::string(key).size()); };
        if (t == "cancel") doCancel = true;
        else if (t == "weld") useWeld = true;
        else if (t.rfind("axis=", 0) == 0) axisFile = val("axis=");
        else if (t.rfind("trend=", 0) == 0) trendMode = std::atoi(val("trend=").c_str());
        else if (t.rfind("area=", 0) == 0) areaPct = std::atof(val("area=").c_str());
        else if (t.rfind("thr=", 0) == 0) thr = std::atof(val("thr=").c_str());
        else if (t.rfind("local=", 0) == 0) useLocal = std::atoi(val("local=").c_str());
        else if (t.rfind("win=", 0) == 0) localWin = std::atof(val("win=").c_str());
        else if (t.rfind("lthr=", 0) == 0) localThr = std::atof(val("lthr=").c_str());
        else if (t.rfind("bexcl=", 0) == 0) bexcl = std::atoi(val("bexcl=").c_str());
        else if (t.rfind("heat=", 0) == 0) heatField = std::atoi(val("heat=").c_str());
        else if (t.rfind("minpts=", 0) == 0) minPts = std::atoi(val("minpts=").c_str());
        else if (t.rfind("ctol=", 0) == 0) cTol = std::atof(val("ctol=").c_str());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) < 0) {
        std::cerr << "点云加载失败: " << argv[1] << "\n";
        return 1;
    }
    std::cout << "点数 = " << cloud->size() << ", 设计半径 = " << designR << " mm\n" << std::endl;

    // ---------- 1. 定半径圆柱寻优(模拟"二次优化圆柱"的第二阶段) ----------
    Eigen::Vector3f ap, ad;
    double secSearch = 0.0;
    bool axisCached = false;
    if (!axisFile.empty()) {
        std::ifstream fin(axisFile);
        if (fin) {
            double v[6];
            for (double& x : v) fin >> x;
            if (fin) {
                ap = Eigen::Vector3f(static_cast<float>(v[0]), static_cast<float>(v[1]),
                                     static_cast<float>(v[2]));
                ad = Eigen::Vector3f(static_cast<float>(v[3]), static_cast<float>(v[4]),
                                     static_cast<float>(v[5]));
                axisCached = true;
            }
        }
    }

    if (!axisCached) {
        MeasureCylindricity ev;
        ev.setInputCloud(cloud);
        ev.setDesignRadius(designR);
        ev.setTolerance(1.0);
        ev.setMaxIterations(2000);
        ev.setVerbose(false);

        if (useWeld) {
            ev.setWeldThresholdFactor(3.0);
            ev.setWeldClusterTolerance(2.0);
            ev.setWeldMinClusterSize(50);
            ev.setWeldConstraintWeight(1.0);
        }
        int cbCount = 0;
        std::string lastStage;
        int lastCur = 0, lastTotal = 0;
        ev.setProgressCallback([&](int cur, int total, const std::string& stage) -> bool {
            ++cbCount;
            lastStage = stage; lastCur = cur; lastTotal = total;
            if (cbCount <= 3) {
                std::cout << "  [进度回调#" << cbCount << "] " << stage
                          << " " << cur << "/" << total << std::endl;
            }
            if (doCancel && cbCount >= 20) {
                std::cout << "  [进度回调#" << cbCount << "] 模拟用户点击取消" << std::endl;
                return false;
            }
            return true;
        });

        const auto t0 = std::chrono::steady_clock::now();
        auto res = useWeld ? ev.evaluateCylindricityWithWeld() : ev.evaluateCylindricity();
        const auto t1 = std::chrono::steady_clock::now();
        secSearch = std::chrono::duration<double>(t1 - t0).count();
        std::cout << "[进度] 回调总次数 = " << cbCount
                  << ", 末次 = " << lastStage << " " << lastCur << "/" << lastTotal << std::endl;
        std::cout << "[取消] isCancelled() = " << (ev.isCancelled() ? "true" : "false") << std::endl;
        if (ev.isCancelled()) {
            std::cout << "[取消] 消息: " << res.assessment_message
                      << ", 耗时 " << secSearch << " s" << std::endl;
            return 0;
        }

        std::cout << std::setprecision(9);
        ap = res.getCylinderAxisPoint();
        ad = res.getCylinderAxisDirection();
        std::cout << "[性能] 定半径粗到精搜索+全量评估: " << secSearch << " s" << std::endl;
        std::cout << "[结果] 轴线点 = (" << ap.x() << ", " << ap.y() << ", " << ap.z() << ")\n";
        std::cout << "[结果] 轴向   = (" << ad.x() << ", " << ad.y() << ", " << ad.z() << ")\n";
        std::cout << "[结果] RMS = " << res.rms_error << " mm, 最大偏差 = "
                  << res.max_deviation << " mm, 不贴合比例 = " << res.outlier_ratio * 100 << "%\n"
                  << std::endl;
        if (!axisFile.empty()) {
            std::ofstream fout(axisFile);
            fout << std::setprecision(9) << ap.x() << " " << ap.y() << " " << ap.z() << " "
                 << ad.x() << " " << ad.y() << " " << ad.z() << "\n";
            std::cout << "[轴线缓存] 已写入 " << axisFile << std::endl;
        }
    }
    else {
        std::cout << "[轴线缓存] 使用 " << axisFile << " (跳过寻优)\n"
                  << "[结果] 轴线点 = (" << ap.x() << ", " << ap.y() << ", " << ap.z() << ")\n"
                  << "[结果] 轴向   = (" << ad.x() << ", " << ad.y() << ", " << ad.z() << ")\n"
                  << std::endl;
    }

    // ---------- 2. 凹塘测量(使用上一步的轴 + 理想半径) ----------
    pcl::ModelCoefficients::Ptr cyl(new pcl::ModelCoefficients);
    cyl->values.resize(7);
    cyl->values[0] = ap.x(); cyl->values[1] = ap.y(); cyl->values[2] = ap.z();
    cyl->values[3] = ad.x(); cyl->values[4] = ad.y(); cyl->values[5] = ad.z();
    cyl->values[6] = static_cast<float>(designR);

    MeasurePothole mp;
    mp.setInputCloud(cloud);
    mp.setCylinder(cyl);
    mp.setDistanceThreshold(thr);   // 0=自动阈值
    mp.setClusterTolerance(cTol);   // 0=自动容差
    mp.setMinClusterSize(minPts);
    mp.setTrendMode(trendMode);
    mp.setMaxAreaFraction(areaPct / 100.0);
    mp.setUseLocalBaseline(useLocal != 0);
    mp.setLocalWindow(localWin);
    mp.setLocalThreshold(localThr);
    mp.setBoundaryExclude(bexcl != 0);
    mp.setHeatMapField(heatField);
    mp.setVerbose(true);

    const auto t2 = std::chrono::steady_clock::now();
    auto pit = mp.evaluate();
    const auto t3 = std::chrono::steady_clock::now();
    const double secPit = std::chrono::duration<double>(t3 - t2).count();

    std::cout << "\n[性能] 凹塘测量耗时: " << secPit << " s" << std::endl;
    std::cout << pit.assessment_message << std::endl;

    // 机器可读的关键数值(便于改造前后逐位对比)
    std::cout << std::setprecision(9);
    std::cout << "[KEY] fit_ok=" << pit.fit_ok << " valid=" << pit.valid              << " max_depth=" << pit.max_depth
              << " pit_points=" << pit.pit_points
              << " cluster_count=" << pit.cluster_count
              << " mean_depth=" << pit.mean_depth
              << " major=" << pit.major_axis << " minor=" << pit.minor_axis
              << " sigma=" << pit.robust_sigma
              << " deepest=(" << pit.deepest_point.x << "," << pit.deepest_point.y
              << "," << pit.deepest_point.z << ")"
              << " area_frac=" << pit.pit_area_fraction
              << " touch_bnd=" << pit.pit_touch_boundary
              << " trend_p2p=" << pit.trend_p2p
              << " trend_removed=" << pit.trend_removed
              << " judged_by_local=" << pit.judged_by_local
              << " local_max_depth=" << pit.local_max_depth
              << " local_threshold=" << pit.local_threshold
              << " baseline_offset_p2p=" << pit.baseline_offset_p2p
              << " bnd_excl_frac=" << pit.boundary_excluded_fraction
              << " local_deepest=(" << pit.local_deepest_point.x << "," << pit.local_deepest_point.y
              << "," << pit.local_deepest_point.z << ")"
              << " heatmap_field_used=" << mp.getHeatMapFieldUsed()
              << std::endl;

    // 多凹坑: 逐个输出机器可读行(供 compare_multi_pit.py 与真值比对)
    // 注意: 展开域坐标 (a, s) 依赖 t1/t2 的具体取向; 本样本的拟合轴与生成脚本的轴
    // 方向相反, 实测 (a_cpp, s_cpp) = (-a_truth, -s_truth), 比较脚本按此对齐.
    std::cout << "[PITCOUNT] " << pit.pits.size()
              << " cluster_count=" << pit.cluster_count
              << " cell=" << pit.grid_cell
              << " win=" << pit.local_window << std::endl;
    for (const auto& it : pit.pits) {
        std::cout << "[PIT] idx=" << it.index
                  << " main=" << (it.is_main ? 1 : 0)
                  << " n=" << it.pit_points
                  << " cells=" << it.pit_cells
                  << " local_max=" << it.local_max_depth
                  << " local_mean=" << it.local_mean_depth
                  << " global_max=" << it.global_max_depth
                  << " major=" << it.major_axis
                  << " minor=" << it.minor_axis
                  << " major_raw=" << it.major_axis_raw
                  << " minor_raw=" << it.minor_axis_raw
                  << " aspect=" << it.aspect_ratio
                  << " ec_a=" << it.ellipse_center_a
                  << " ec_s=" << it.ellipse_center_s
                  << " cen_a=" << it.centroid_a
                  << " cen_s=" << it.centroid_s
                  << " span_a=" << it.contour_span_a
                  << " span_s=" << it.contour_span_s
                  << " valid=" << (it.valid ? 1 : 0)
                  << " ell_ok=" << (it.ellipse_ok ? 1 : 0)
                  << " reason=" << (it.reject_reason.empty() ? std::string("-") : it.reject_reason)
                  << std::endl;
    }

    // 凹塘点群质心在展开域(a=轴向, s=周向弧长)的坐标: 用于与真值中心比对
    // (最深点是"最深的那个点", 在平底坑上对基准抖动敏感; 质心更能代表坑中心)
    {
        auto pitCloud = mp.getPitCloud();
        if (pitCloud && !pitCloud->empty()) {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();   // 必须用 double: 3D 坐标量级 1e3mm x 1e5 点
            for (const auto& p : pitCloud->points) sum += Eigen::Vector3d(p.x, p.y, p.z);
            const Eigen::Vector3d c = sum / static_cast<double>(pitCloud->size());
            const Eigen::Vector3d rel = c - ap.cast<double>();
            const Eigen::Vector3d adD = ad.cast<double>();
            const double a = rel.dot(adD);
            Eigen::Vector3d a0 = Eigen::Vector3d::UnitX();
            if (std::abs(a0.dot(adD)) > 0.9) a0 = Eigen::Vector3d::UnitY();
            Eigen::Vector3d t1 = a0 - adD * a0.dot(adD);
            t1.normalize();
            const Eigen::Vector3d t2 = adD.cross(t1);
            const double s = designR * std::atan2(rel.dot(t2), rel.dot(t1));
            std::cout << std::setprecision(9)
                      << "[PITCENTER] n=" << pitCloud->size()
                      << " a=" << a << " s=" << s
                      << " cen3d=(" << c.x() << "," << c.y() << "," << c.z() << ")" << std::endl;
        }
    }
    return 0;
}
