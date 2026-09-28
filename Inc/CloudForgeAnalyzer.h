#pragma once
#include "config/qt6.h"
#include "config/pcl114.h"
#include "config/vtk9.h"
#include "Fitting/Fitting.h"
#include "PreProcessing/PreProcessing.h"
#include "Measure/Measure.h"
#include "Basic/Basic.h"
#include "UndoRedoManager.h"


#include <string>
#include <filesystem>
#include <atomic>
#include <mutex>
#include <memory>
#include <functional>
#include <QSemaphore>
#include <QThread>
#include "Protrusion_Depression_Cylinder.h"
#include "Linear_Depression_Plane.h"




QT_BEGIN_NAMESPACE
namespace Ui { class CloudForgeAnalyzerClass; };
QT_END_NAMESPACE

class CloudForgeAnalyzer : public QMainWindow
{
    Q_OBJECT

public:
    CloudForgeAnalyzer(QWidget *parent = nullptr);
    ~CloudForgeAnalyzer();
    Ui::CloudForgeAnalyzerClass* ui;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private slots:
    //void Slot_CurveFittingPC();
    void Slot_ChangeVA_x();
    void Slot_ChangeVA_y();
    void Slot_ChangeVA_z();
    void Slot_ChangeVA_o();
    void Slot_fi_open_Triggered();
    void Slot_fi_openSTL_Triggered();
    void Slot_fi_save_Triggered();
    void Slot_fi_saveas_Triggered();
    void Slot_fi_add_Triggered();
    void Slot_ph_1_Triggered();
    void Slot_fl_1_Triggered();
    void Slot_fl_2_Triggered();
    void Slot_ed_dork_Triggered();
    void Slot_ed_cleangeo_Triggered();
    void Slot_ed_cleanall_Triggered();
    void Slot_ed_cleanRGB_Triggered();
    void Slot_ed_cleangeodetic_Triggered();
    void Slot_ed_clean2DActor_Triggered();
    void Slot_ed_undo_Triggered();
    void Slot_ed_redo_Triggered();
    void Slot_fit_cy_Triggered();
    void Slot_fit_cy2_Triggered();
    void Slot_fit_cy3_Triggered();
    void Slot_fit_plane_Triggered();
    void Slot_fit_line_Triggered();
    void Slot_ph_CurvSeg_Triggered();
    void Slot_ph_ProtruSeg_Triggered();

    void Tool_MeasureArc();
    void Tool_MeasureGeodisic();
    void Tool_MeasureHeight();
    void Tool_MeasureParallel();
    void Tool_MeasurePlanarity();
    void Tool_MeasureAngleP2P();
    void Tool_Clip();
    void Tool_MeasureCylindricity();
    void Tool_MeasureWeldHeight();
    void Tool_MeasurePothole();
    void Update_PointCounts();
private:
    // 焊前装配阶差/间隙的共同流程(§7): 选两件 → 定接缝区域 → 看 Z 均值 → 设 R 与方向 → 后台计算
    // 口径(2026-09 统一): 阶差 = 径向(e_P - e_Q, 近件在外为正); 间隙 = 沿跨缝方向(环缝=轴向)
    // 两个量由同一次配对同时得到, 只有一个入口; 结果只输出文本报告, 不做三维标注。
    void Tool_MeasureWeldPreparation();
    // 焊前装配测量的三维标注(只在 GUI 线程; 数据来自 Result 与参考圆柱框架)
    void VisualizeWeldPreparation(const MeasureWeldPreparation::Result& result,
                                  const CylinderSurfaceFrame& frame);
    std::vector<std::string> m_weldPrepShapeIds;   // PCL 形状(三维文字)
    std::vector<std::string> m_weldPrepLineIds;    // 直线段(走 LineMap 注册)
    std::vector<std::string> m_weldPrepActorIds;   // vtkActor(折线)
    void cleanWeldPrepVisuals();

    //  
    void visualizeFittedPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const pcl::ModelCoefficients::Ptr& plane_coeffs,
        const std::string& plane_id,
        double r = 0.0, double g = 1.0, double b = 1.0, // Ĭ    ɫ
        double opacity = 0.3);

    void addCylinderResult(const std::string& name, pcl::ModelCoefficients::Ptr coeff);
    void addPlaneResult(const std::string& name, pcl::ModelCoefficients::Ptr coeff);
    pcl::ModelCoefficients::Ptr getCylinderResult(const std::string& name);
    bool removeCylinderResult(const std::string& name);
    void clearAllCylinderResults();
    void clearAllActors();
    void AddActors(std::string id, vtkSmartPointer<vtkActor> actor);
    std::vector<std::string> getAllCylinderNames();

    void AddPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ColorManager color);
    void ClearAllPointCloudRGB();
    void ClearAllPointCloud();
    void DelePointCloud(std::string name);

    // 撤销/重做相关
    void saveUndoState(const std::string& description);
    void rebuildCloudVisualization();
    void beginUndoBatch(const std::string& description);
    void endUndoBatch();

    void InitalizeQWidgets();
    void InitalizeRenderer();
    void InitalizeConnects();
    void TeEDebug(std::string debugMes);
    void UpdateCamera(int a, int b, int c);
    void Update_CFmes(std::string cfmes);
    void mainLoop_Init();
    //          
    void InitializeProgressBar();
    void SetProgressBarValue(int percentage, const QString& message = "");
    void ResetProgressBar();

    // ============ 后台计算任务(避免界面无响应) ============
    // 计算在工作线程执行; 进度经原子共享状态传回, GUI 线程用定时器读取并刷新
    // 进度条/调试文本框; VTK 可视化仍只在 GUI 线程执行。
    struct AsyncTaskState {
        std::atomic<bool> cancelRequested{ false };
        std::atomic<int> current{ 0 };
        std::atomic<int> total{ 0 };
        std::mutex mtx;
        std::string stage;
        std::vector<std::string> pendingLog;
    };

    void BeginAsyncTask(const QString& title);
    void EndAsyncTask(bool cancelled);
    void PollAsyncProgress();
    void CancelCurrentTask();
    bool IsTaskRunning() const { return m_asyncRunning; }

    // 以下两个可从工作线程调用(只写共享状态, 不触碰 Qt 对象)
    bool PostProgress(int current, int total, const std::string& stage);
    void PostLog(const std::string& line);

    // 执行后台任务: work 在工作线程, onFinished 在 GUI 线程(此时任务已收尾)
    void RunAsyncVoid(const QString& title,
                      const std::function<void()>& work,
                      const std::function<void()>& onFinished = nullptr);

    // 圆柱拟合流程的异步分段(初次拟合 → 可视化/参数 → 优化 → 可视化/保存)
    void ContinueFitCy2AfterInitialFit(std::shared_ptr<Fit_Cylinder> fcy,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp);
    void FinishFitCy2(const std::shared_ptr<MeasureCylindricity::AssessmentResult>& result,
                      const std::shared_ptr<MeasureCylindricity>& evaluator,
                      const Eigen::Vector3f& initial_center,
                      const Eigen::Vector3f& initial_axis,
                      double design_radius);
    void ContinueFitCy3AfterInitialFit(std::shared_ptr<Fit_Cylinder> fcy,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp);
    void FinishFitCy3(const std::shared_ptr<MeasureCylindricity::AssessmentResult>& result,
                      const std::shared_ptr<MeasureCylindricity>& evaluator,
                      const Eigen::Vector3f& initial_center,
                      const Eigen::Vector3f& initial_axis,
                      double design_radius);

    // STL 导入流程的异步分段(后台读取文件信息 → 界面弹参数 → 后台转换 → 界面加入场景)
    struct StlImportInfo {
        int triangleCount = 0;            // STL 三角形数量
        float modelDiagonal = 0.0f;       // 模型包围盒对角线(米)
        float recommendedLeafSize = 0.01f;// 推荐的降采样 leaf-size
        QString fileName;                 // 文件名(不含路径)
        double fileSizeMB = 0.0;          // 文件大小(MB)
        QString error;                    // 非空表示无法读取STL文件(致命错误)
    };
    struct StlImportResult {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
        bool ok = false;                  // 转换是否成功
        QString error;                    // 非空表示失败原因
    };
    void ContinueStlImportAfterInfo(const std::shared_ptr<StlImportInfo>& info,
                                    const std::string& stlPath);
    void FinishStlImport(const std::shared_ptr<StlImportInfo>& info,
                         const std::shared_ptr<StlImportResult>& result,
                         float leafSize, bool surfaceOnly);

    std::shared_ptr<AsyncTaskState> m_asyncState;
    QTimer* m_asyncTimer = nullptr;
    QFuture<void> m_asyncFuture;
    bool m_asyncRunning = false;
    std::atomic<bool> m_shuttingDown{ false };

    // 新增：长耗时计算的进度回调工厂(供测量类在后台线程中回报进度)
    std::function<bool(int, int, const std::string&)> WorkerProgressCallback();

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr renderer_custom;
    UndoRedoManager m_undoRedoManager;
    int m_undoBatchLevel = 0; // >0 时 AddPointCloud 等不单独记录 undo
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudMap;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RGBCloudMap;
    std::map<std::string, ColorManager> ColorMap;
    std::map<std::string, Line> LineMap;
    std::vector<vtkSmartPointer<vtkProp>> m_geodesicVisualizationActors;
    std::map<std::string, vtkSmartPointer<vtkActor>> m_arcSplineMap;
    std::map<std::string, pcl::ModelCoefficients::Ptr> cylinderResultsMap;
    std::map<std::string, pcl::ModelCoefficients::Ptr> planeResultsMap;
    std::map<std::string, vtkSmartPointer<vtkActor>> ActorMap;

    // 焊缝高度测量形状追踪（用于撤销时清理）
    std::vector<std::string> m_weldMeasureShapeIds;
    void cleanWeldMeasureVisuals();

    void cleanGeodesicVisualization();
    void AddLine(const std::string& name,
        const pcl::PointXYZ& start,
        const pcl::PointXYZ& end,
        const ColorManager& color,
        double width = 2.0,
        Eigen::VectorXf coeffs= Eigen::VectorXf());

    void DeleteLine(const std::string& name);
    void ClearAllLines();
    static void visualizeMeasurementResults(MeasureHeight& measurer,
        pcl::PointCloud<pcl::PointXYZ>::Ptr measureCloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud);
    static void colorPointCloudByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::ModelCoefficients::Ptr plane_coeffs,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);

    static double calculatePlaneSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    static void addHeightLines(pcl::visualization::PCLVisualizer::Ptr viewer,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::ModelCoefficients::Ptr plane_coeffs);

    void visualizeCylindricityHeatMap(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud,
        double min_distance, double max_distance);
    void visualizePlanarityHeatMap(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud,
        double min_distance, double max_distance,
        const std::string& plane_name);
    bool showConfirmationDialog(const QString& title, const QString& message);

    void addArcSplineActor(const std::string& id, vtkSmartPointer<vtkActor> actor);
    bool removeArcSplineActor(const std::string& id);
    void clearAllArcSplineActors();
    std::vector<std::string> getAllArcSplineIds() const;
    vtkSmartPointer<vtkActor> getArcSplineActor(const std::string& id);
};
