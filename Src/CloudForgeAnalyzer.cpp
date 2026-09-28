#include "CloudForgeAnalyzer.h"
#include "Dialog/ParamDialogMeasureWeldHeight.h"
#include <thread>
#include <omp.h> // OpenMP 并行: MSVC(/openmp) 与 GCC(-fopenmp) 双平台兼容
#include <random> // 线程私有随机数 (std::mt19937)
#include <QElapsedTimer>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

CloudForgeAnalyzer::CloudForgeAnalyzer(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::CloudForgeAnalyzerClass())
    , cloud(new pcl::PointCloud<pcl::PointXYZ>)
    , renderer_custom(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(255, 255, 255))
{
    //qt控件区head
    ui->setupUi(this);
    InitalizeQWidgets();
    InitalizeConnects();
    InitalizeRenderer();
    mainLoop_Init();
    if (pcl::io::loadPCDFile("PCDfiles/rabbit.pcd", *cloud) == -1) {
        TeEDebug(">>qt构造:无法加载点云文件");
        return;
    }
    ColorManager color(255,255,255);
    ++m_undoBatchLevel; // 初始化加载不记入撤销历史
    AddPointCloud("example", cloud, color);
    --m_undoBatchLevel;

}

CloudForgeAnalyzer::~CloudForgeAnalyzer()
{
    // 等待后台计算结束, 避免工作线程访问已析构对象
    m_shuttingDown.store(true);
    if (m_asyncState) {
        m_asyncState->cancelRequested.store(true);
    }
    if (m_asyncFuture.isRunning()) {
        m_asyncFuture.waitForFinished();
    }
    viewer.reset();
    delete ui;
}



void CloudForgeAnalyzer::InitalizeRenderer() {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    // 创建 PCLVisualizer（使用上面创建的 renderer/renderWindow）
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    viewer->getRenderWindow()->GlobalWarningDisplayOff();

    // 先把 renderWindow 绑定到 QVTK widget，让 QVTK widget 创建并管理其 interactor                                                                                                                                                                
    ui->winOfAnalyzer->setRenderWindow(viewer->getRenderWindow());
    QApplication::processEvents();
    ui->winOfAnalyzer->makeCurrent();

    // 现在安全地从 widget 获取 interactor 并交给 PCLVisualizer
    vtkRenderWindowInteractor* interactor = ui->winOfAnalyzer->interactor();
    if (interactor) {
        viewer->setupInteractor(interactor, ui->winOfAnalyzer->renderWindow());
    }
    else {
        qWarning() << "初始化渲染器：无法从 QVTK widget 获取 interactor（可能会导致交互异常）";
    }

    viewer->setBackgroundColor(0, 0.3, 0.4);
    viewer->addCoordinateSystem(4.0);
}

void CloudForgeAnalyzer::InitalizeQWidgets() {
    InitializeProgressBar();
}

void CloudForgeAnalyzer::InitalizeConnects() {
    connect(ui->action_ed_dork, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_dork_Triggered);
    connect(ui->action_ed_cleangeo, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_cleangeo_Triggered);
    connect(ui->action_ed_cleangall, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_cleanall_Triggered);
    connect(ui->action_ed_cleanRGB, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_cleanRGB_Triggered);
    connect(ui->action_ed_cleangeodetic, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_cleangeodetic_Triggered);
    connect(ui->action_ed_clean2DActor, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_clean2DActor_Triggered);
    connect(ui->action_ed_undo, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_undo_Triggered);
    connect(ui->action_ed_redo, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_redo_Triggered);
    connect(ui->action_fi_open, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_open_Triggered);
    connect(ui->action_fi_openSTL, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_openSTL_Triggered);
    connect(ui->action_fi_save, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_save_Triggered);
    connect(ui->action_fi_saveas, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_saveas_Triggered);
    connect(ui->action_fi_add, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_add_Triggered);
    connect(ui->action_ph_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_1_Triggered);
    connect(ui->action_CurvSeg, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_CurvSeg_Triggered);
    connect(ui->action_ProtruSeg, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_ProtruSeg_Triggered);
    connect(ui->action_fl_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_1_Triggered);
    connect(ui->action_fl_2, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_2_Triggered);
    connect(ui->action_fit_cy, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_cy_Triggered);
    connect(ui->action_fit_cy2, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_cy2_Triggered);
    connect(ui->action_fit_cy3, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_cy3_Triggered);
    connect(ui->action_fit_line, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_line_Triggered);
    connect(ui->action_fit_plane, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_plane_Triggered);
    connect(ui->measure_cylinder, &QAction::triggered,this, &CloudForgeAnalyzer::Tool_MeasureArc);
    connect(ui->measure_geodisic, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureGeodisic);
    connect(ui->measure_parallel, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureParallel);
    connect(ui->measure_height, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureHeight);
    connect(ui->measure_planarity, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasurePlanarity);
    connect(ui->measure_angleP2P, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureAngleP2P);
    connect(ui->measure_Cylindricity, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureCylindricity);
    connect(ui->measure_weldheight, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureWeldHeight);
    connect(ui->measure_pothole, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasurePothole);
    connect(ui->measure_weld_prep, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureWeldPreparation);
    connect(ui->action_cancel_task, &QAction::triggered, this, &CloudForgeAnalyzer::CancelCurrentTask);
    ui->action_cancel_task->setEnabled(false);   // 无任务时不可用
    connect(ui->action_Clip, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_Clip);
}

void CloudForgeAnalyzer::mainLoop_Init() {
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(Update_PointCounts())); // slotCountMessage是我们需要执行的响应函数 
    timer->start(200); // 每隔1s 
}


bool CloudForgeAnalyzer::showConfirmationDialog(const QString& title, const QString& message){
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, title, message,
        QMessageBox::Yes | QMessageBox::No);
    return (reply == QMessageBox::Yes);
}

////////////////////////////////////////////////////////////////////////////////////////////////*槽函数start*/
void CloudForgeAnalyzer::Slot_fit_plane_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择点云进行平面拟合");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    // 参数对话框在界面线程弹出(原先在 Fit_Plane 构造函数里弹框)
    ParamDialog_FittingPlane fitDialog;
    if (fitDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3, ok4;
    Fit_Plane::FitParams fp;
    fp.LocalRadius = fitDialog.getParams()[0].toFloat(&ok1);
    fp.AnomalyThreshold = fitDialog.getParams()[1].toFloat(&ok2);
    fp.PlaneFitThreshold = fitDialog.getParams()[2].toFloat(&ok3);
    fp.MaxIterations = fitDialog.getParams()[3].toInt(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行平面拟合(异常点检测+RANSAC), 完成后回到界面线程可视化
    auto fpHolder = std::make_shared<std::shared_ptr<Fit_Plane>>();
    RunAsyncVoid("平面拟合",
        [this, fpHolder, Cloud_Temp, fp]() {
            if (!PostProgress(0, 0, "平面拟合: 异常点检测与RANSAC...")) {
                PostLog(">>: 平面拟合已取消。");
                return;
            }
            // 计算类在工作线程中构造(参数构造函数只保存参数, 点云拷贝也在此完成)并执行
            *fpHolder = std::make_shared<Fit_Plane>(Cloud_Temp, fp);
            (*fpHolder)->compute();
            PostLog((*fpHolder)->message);
            PostProgress(100, 100, "平面拟合完成");
        },
        [this, fpHolder, Cloud_Temp]() {
    if (!*fpHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 平面拟合已取消，不再显示本次结果。");
        return;
    }
    Fit_Plane& planeFitter = **fpHolder;
    if (planeFitter.isCancelled) {
        TeEDebug(">>: 平面拟合操作取消");
        return;
    }
    if (planeFitter.Get_Coeff_in().size() < 4) {
        TeEDebug(">>: 平面拟合失败：未能获得有效的平面模型系数");
        Update_CFmes(planeFitter.message);
        return;
    }
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_inliers = planeFitter.Get_Inliers();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_outliers = planeFitter.Get_Outliers();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_anomaly = planeFitter.Get_AnomalyPoints();

	ColorManager color_inliers(0, 255, 0);   // 绿色-内点
	ColorManager color_outliers(255, 0, 0);  // 红色-外点
	ColorManager color_anomaly(255, 255, 0); // 黄色-异常点

	beginUndoBatch("平面拟合");
	AddPointCloud("plane_fit_inliers", Cloud_inliers, color_inliers);
	AddPointCloud("plane_fit_outliers", Cloud_outliers, color_outliers);
    AddPointCloud("plane_fit_anomaly", Cloud_anomaly, color_anomaly);
	endUndoBatch();

	std::string message = planeFitter.message;
    Eigen::Vector4f coeff_vec = planeFitter.Get_Coeff_in(); // [A, B, C, D]

    // 3. 直接转换为 pcl::ModelCoefficients::Ptr 并存储
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients());
    plane_coeff->values.push_back(coeff_vec[0]);
    plane_coeff->values.push_back(coeff_vec[1]);
    plane_coeff->values.push_back(coeff_vec[2]);
    plane_coeff->values.push_back(coeff_vec[3]);
	std::string planeName = GenerateRandomName("fitted_plane");
	addPlaneResult(planeName, plane_coeff);
    visualizeFittedPlane(Cloud_Temp,plane_coeff, planeName);
	Update_CFmes(message);
	TeEDebug(">>: 平面拟合完成。内点(绿)/外点(红)已可视化。");
        });
}

void CloudForgeAnalyzer::Tool_MeasurePlanarity() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // 1. 选择待评估的点云
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择待评估平面度的点云");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = CloudMap[dialog.getSelectedList()[0]];

    // 2. 选择已有的平面拟合结果作为评估基准
    if (planeResultsMap.empty()) {
        TeEDebug(">>: 没有可用的平面拟合结果，请先进行平面拟合");
        return;
    }

    ChosePlaneDialog dialog1(planeResultsMap);
    if (dialog1.getSelectedList().empty()) {
        TeEDebug(">>: 未选择平面拟合结果");
        return;
    }

    std::string selectedPlaneName = dialog1.getSelectedList()[0];
    pcl::ModelCoefficients::Ptr selected_plane = planeResultsMap[selectedPlaneName];
    std::string planeName = GenerateRandomName("chosen_plane");
    addPlaneResult(planeName, selected_plane);
    visualizeFittedPlane(target_cloud, selected_plane, planeName);

    // 可视化参考平面
    std::string visualization_id = "measurement_ref_" + selectedPlaneName;
    viewer->addPlane(*selected_plane, visualization_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        0.0, 1.0, 1.0, // 青色，以示区别
        visualization_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
        0.3, // 半透明
        visualization_id);
    TeEDebug(">>: 参考平面 '" + selectedPlaneName + "' 已可视化(青色半透明)。");

    // 3. 从选择的平面结果中提取平面参数
    std::vector<float> plane_coeffs = {
        selected_plane->values[0], // A
        selected_plane->values[1], // B
        selected_plane->values[2], // C
        selected_plane->values[3]  // D
    };


    // 5. 创建评估器并设置参数, 在后台线程执行评估(界面保持响应)
    auto evaluator = std::make_shared<MeasurePlanarity>();
    auto resultPtr = std::make_shared<MeasurePlanarity::AssessmentResult>();

    RunAsyncVoid("平面度评估",
        [this, evaluator, resultPtr, target_cloud, selected_plane]() {
            PostProgress(0, 0, "平面度评估: 逐点计算到基准平面的距离...");
            evaluator->setInputCloud(target_cloud);
            evaluator->setPlaneParameters(selected_plane);
            *resultPtr = evaluator->evaluatePlanarity();
            PostLog(resultPtr->assessment_message);
        },
        [this, evaluator, resultPtr, selectedPlaneName]() {
    const auto& result = *resultPtr;

    // 7. 生成并可视化热力图
    auto heatmap_cloud = evaluator->getHeatMapCloud();
    double min_distance, max_distance;
    evaluator->getDistanceRange(min_distance, max_distance);

    // 调用自定义热力图可视化函数
    visualizePlanarityHeatMap(heatmap_cloud, min_distance, max_distance, selectedPlaneName);

    // 8. 显示评估结果
    qDebug() << result.assessment_message;
    TeEDebug(result.assessment_message);
    Update_CFmes(result.assessment_message);

    // 9. 刷新视图
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
        });
}

void CloudForgeAnalyzer::visualizePlanarityHeatMap(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud,
    double min_distance, double max_distance,
    const std::string& plane_name)
{
    TeEDebug("开始可视化平面度热力图...");
    if (!heatmap_cloud || heatmap_cloud->empty()) {
        TeEDebug("热力图点云为空，无法可视化");
        return;
    }

    std::string heatmap_id = "planarity_heatmap_" + plane_name;
    std::string colorbar_id = "planarity_colorbar_" + plane_name;

    // 先清除可能存在的旧热力图及标注
    viewer->removePointCloud(heatmap_id);
    viewer->removeShape(colorbar_id);
    viewer->removeText3D(heatmap_id + "_title");

    // 添加热力图点云
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(heatmap_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(heatmap_cloud, rgb, heatmap_id);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, heatmap_id);

    // 添加到统一管理变量（参照visualizeCylindricityHeatMap）
    RGBCloudMap.emplace(heatmap_id, heatmap_cloud);

    // === 创建并添加固定颜色条 (Colorbar) ===
    // 1. 获取当前渲染器
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        TeEDebug("错误：无法获取渲染器，颜色条创建失败。");
    }
    else {
        // 2. 创建颜色查找表 (Lookup Table)，映射从绿到红
        vtkNew<vtkLookupTable> hueLut;
        hueLut->SetTableRange(min_distance, max_distance); // 标量值范围对应距离范围
        hueLut->SetHueRange(0.33, 0.0);      // 从绿色 (0.33) 到红色 (0.0)
        hueLut->SetSaturationRange(1, 1);
        hueLut->SetValueRange(1, 1);
        hueLut->SetNanColor(1, 1, 1, 1);    // 无效值显示为白色
        hueLut->SetNumberOfTableValues(256); // 颜色精度
        hueLut->Build();

        // 3. 创建颜色条 Actor
        vtkNew<vtkScalarBarActor> scalarBar;
        scalarBar->SetLookupTable(hueLut);
        scalarBar->SetTitle("mm");
        scalarBar->SetNumberOfLabels(5); // 主标签数量
        scalarBar->SetMaximumNumberOfColors(256);

        const int titleFontSize = 12;        // 颜色条标题文字大小
        const int labelFontSize = 15;        // 颜色条标签文字大小
        const double colorbarWidth = 0.05;   // 颜色条宽度 (占窗口宽度的比例)
        const double colorbarHeight = 0.6;   // 颜色条高度 (占窗口高度的比例)
        const double colorbarPosX = 0.92;    // 颜色条右侧位置
        const double colorbarPosY = 0.2;     // 颜色条底部位置

        // 设置颜色条文本属性
        vtkNew<vtkTextProperty> titleProperty;
        titleProperty->SetFontSize(titleFontSize);
        titleProperty->BoldOn();
        titleProperty->SetColor(1, 1, 1); // 白色
        scalarBar->SetTitleTextProperty(titleProperty);
        scalarBar->SetTitleRatio(0.6);

        vtkNew<vtkTextProperty> labelProperty;
        labelProperty->SetFontSize(labelFontSize);
        labelProperty->SetColor(0.9, 0.9, 0.9); // 浅灰色
        scalarBar->SetLabelTextProperty(labelProperty);

        // 应用位置和大小
        scalarBar->SetPosition(colorbarPosX - colorbarWidth, colorbarPosY);
        scalarBar->SetWidth(colorbarWidth);
        scalarBar->SetHeight(colorbarHeight);
        scalarBar->SetPickable(0); // 禁止拾取，避免与点云交互冲突

        // 4. 将颜色条添加到渲染器
        renderer->AddActor2D(scalarBar);

    }
    // === 颜色条添加结束 ===

    TeEDebug("平面度热力图与颜色条可视化完成");
    TeEDebug("距离范围: " + std::to_string(min_distance) + " - " + std::to_string(max_distance) + " mm");

    // 刷新渲染窗口
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}


void CloudForgeAnalyzer::Slot_fit_cy2_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // 1. 选择点云
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择点云进行圆柱拟合与优化");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    // 2. 先收集初始拟合参数（原先在 Fit_Cylinder 构造函数里弹框，移到界面线程）
    ParamDialog_FittingCylinder fitDialog;
    if (fitDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3, ok4;
    Fit_Cylinder::FitParams fp;
    fp.KSearch = fitDialog.getParams()[0].toInt(&ok1);
    fp.DistanceThreshold = fitDialog.getParams()[1].toFloat(&ok2);
    fp.MaxIterations = fitDialog.getParams()[2].toInt(&ok3);
    fp.InitialRadius = fitDialog.getParams()[3].toFloat(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 3. 后台执行初次拟合(法线估计+RANSAC), 完成后回到界面线程继续
    auto fcy = std::make_shared<Fit_Cylinder>(Cloud_Temp, fp);
    RunAsyncVoid("圆柱初次拟合",
        [this, fcy]() {
            QElapsedTimer tStage1;
            tStage1.start();
            PostProgress(0, 0, "初次拟合: 法线估计与RANSAC...");
            fcy->compute();
            PostLog("[Perf] 第一阶段Fit_Cylinder总耗时: " + std::to_string(tStage1.elapsed()) + " ms");
        },
        [this, fcy, Cloud_Temp]() {
            if (fcy->isCancelled || fcy->Get_Coeff_in().size() < 7) {
                TeEDebug(">>: 初次圆柱拟合未获得有效结果，流程结束。");
                return;
            }
            ContinueFitCy2AfterInitialFit(fcy, Cloud_Temp);
        });
}

// 初次拟合完成后的界面部分: 可视化 + 收集优化参数 + 启动后台优化
void CloudForgeAnalyzer::ContinueFitCy2AfterInitialFit(std::shared_ptr<Fit_Cylinder> fcy,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp)
{
    Eigen::VectorXf coeff1 = fcy->Get_Coeff_in();
    pcl::ModelCoefficients::Ptr cycoeff1(new pcl::ModelCoefficients);
    cycoeff1->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cycoeff1->values[i] = coeff1(i);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Inliers = fcy->Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Outliers = fcy->Get_Outliers();
    ColorManager color_inliers(0, 255, 0);   // 绿色-内点
    ColorManager color_outliers(255, 0, 0);  // 红色-外点
    beginUndoBatch("初始圆柱拟合");
    AddPointCloud("initial_fit_inliers", Cloud_Inliers, color_inliers);
    AddPointCloud("initial_fit_outliers", Cloud_Outliers, color_outliers);
    endUndoBatch();

    viewer->addCylinder(*cycoeff1, "initial_fit_cylinder");
    float line_length = 800.0f;
    Eigen::Vector3f initial_center = fcy->get_center_point();
    Eigen::Vector3f initial_axis = fcy->get_axis_direction();

    // 输出初次拟合信息
    Update_CFmes(fcy->message);
    TeEDebug(">>: 初次圆柱拟合完成。内点(绿)/外点(红)与圆柱体(initial_fit_cylinder)已可视化。");

    ParamDialogMeausreCy pdialog;
    if (pdialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消，二次优化跳过。初次拟合结果已保存。");
        addCylinderResult(GenerateRandomName("fitted_cylinder"), cycoeff1);
        return;
    }

    double design_radius = pdialog.getParams()[0].toDouble();
    double tolerance = pdialog.getParams()[1].toDouble();
    int iters = pdialog.getParams()[2].toInt();

    // 创建圆柱度评估器，并以初次拟合结果为初始值
    auto evaluator = std::make_shared<MeasureCylindricity>();
    evaluator->setInputCloud(Cloud_Temp);
    evaluator->setDesignRadius(design_radius);
    evaluator->setTolerance(tolerance);
    evaluator->setMaxIterations(iters);
    evaluator->setVerbose(false);
    evaluator->setInitialLine(initial_center, initial_axis); // 设置初始值
    evaluator->setProgressCallback(WorkerProgressCallback());

    // 后台执行轴线优化(第二阶段粗到精搜索), 界面保持响应
    auto resultPtr = std::make_shared<MeasureCylindricity::AssessmentResult>();
    RunAsyncVoid("圆柱轴线优化",
        [evaluator, resultPtr]() {
            *resultPtr = evaluator->evaluateCylindricity();
        },
        [this, resultPtr, evaluator, initial_center, initial_axis, design_radius]() {
            if (evaluator->isCancelled()) {
                TeEDebug(">>: 圆柱轴线优化已取消，未保存结果。");
                return;
            }
            FinishFitCy2(resultPtr, evaluator, initial_center, initial_axis, design_radius);
        });
}

// 优化完成后的界面部分: 可视化优化圆柱与轴线 + 保存结果 + 报告
void CloudForgeAnalyzer::FinishFitCy2(const std::shared_ptr<MeasureCylindricity::AssessmentResult>& result,
                                      const std::shared_ptr<MeasureCylindricity>& evaluator,
                                      const Eigen::Vector3f& initial_center,
                                      const Eigen::Vector3f& initial_axis,
                                      double design_radius)
{
    const float line_length = 800.0f;
    Eigen::Vector3f optimized_center = result->getCylinderAxisPoint();
    Eigen::Vector3f optimized_axis = result->getCylinderAxisDirection();

    pcl::ModelCoefficients::Ptr cycoeff2(new pcl::ModelCoefficients);
    cycoeff2->values.resize(7);
    cycoeff2->values[0] = optimized_center.x();
    cycoeff2->values[1] = optimized_center.y();
    cycoeff2->values[2] = optimized_center.z();
    cycoeff2->values[3] = optimized_axis.x();
    cycoeff2->values[4] = optimized_axis.y();
    cycoeff2->values[5] = optimized_axis.z();
    cycoeff2->values[6] = static_cast<float>(design_radius); // 使用设定的设计半径

    viewer->addCylinder(*cycoeff2, "optimized_fit_cylinder");
    optimized_axis.normalize();
    Eigen::Vector3f p1_opt = optimized_center - optimized_axis * line_length;
    Eigen::Vector3f p2_opt = optimized_center + optimized_axis * line_length;

    vtkSmartPointer<vtkLineSource> lineSource2 = vtkSmartPointer<vtkLineSource>::New();
    lineSource2->SetPoint1(p1_opt.x(), p1_opt.y(), p1_opt.z());
    lineSource2->SetPoint2(p2_opt.x(), p2_opt.y(), p2_opt.z());
    lineSource2->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper2->SetInputConnection(lineSource2->GetOutputPort());

    vtkSmartPointer<vtkActor> lineActor2 = vtkSmartPointer<vtkActor>::New();
    lineActor2->SetMapper(mapper2);
    lineActor2->GetProperty()->SetColor(1.0, 0.0, 0.0); //red
    lineActor2->GetProperty()->SetLineWidth(3.0);

    AddActors(GenerateRandomName("axis"), lineActor2);

    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();

    // 5. 存储最终优化结果
    std::string storedName = GenerateRandomName("optimized_cylinder");
    addCylinderResult(storedName, cycoeff2);

    std::stringstream ss;
    ss << "圆柱拟合与优化完成。\n";
    // 此处显示的是优化过程产生的评估信息，仅为参考。正式的评估应由Tool_MeasureCylindricity完成
    ss << "初次拟合轴线点: (" << initial_center.x() << ", "
        << initial_center.y() << ", " << initial_center.z() << ")";
    ss << "初次拟合轴线方向: (" << initial_axis.x() << ", "
        << initial_axis.y() << ", " << initial_axis.z() << ")";
    ss << "二次优化后轴线点: (" << optimized_center.x() << ", "
        << optimized_center.y() << ", " << optimized_center.z() << ")";
    ss << "二次优化轴线方向: (" << optimized_axis.x() << ", "
        << optimized_axis.y() << ", " << optimized_axis.z() << ")";
    std::string finalMsg = "圆柱拟合与优化完成。\n";
    finalMsg += "初次拟合：内点(绿)/外点(红)，圆柱体 'initial_fit_cylinder'\n";
    finalMsg += "二次优化：圆柱体 'optimized_fit_cylinder'\n";

    TeEDebug(">>: 二次优化完成，几何体已更新。");
    Update_CFmes(finalMsg + ss.str());
}

void CloudForgeAnalyzer::Slot_fit_cy3_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // 1. 选择点云（含焊缝的原始壁面点云）
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择点云进行圆柱拟合与焊缝约束优化");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    // 2. 先收集初始拟合参数（原先在 Fit_Cylinder 构造函数里弹框，移到界面线程）
    ParamDialog_FittingCylinder fitDialog;
    if (fitDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3, ok4;
    Fit_Cylinder::FitParams fp;
    fp.KSearch = fitDialog.getParams()[0].toInt(&ok1);
    fp.DistanceThreshold = fitDialog.getParams()[1].toFloat(&ok2);
    fp.MaxIterations = fitDialog.getParams()[2].toInt(&ok3);
    fp.InitialRadius = fitDialog.getParams()[3].toFloat(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 3. 后台执行初次拟合(法线估计+RANSAC), 完成后回到界面线程继续
    auto fcy = std::make_shared<Fit_Cylinder>(Cloud_Temp, fp);
    RunAsyncVoid("圆柱初次拟合",
        [this, fcy]() {
            PostProgress(0, 0, "初次拟合: 法线估计与RANSAC...");
            fcy->compute();
        },
        [this, fcy, Cloud_Temp]() {
            if (fcy->isCancelled || fcy->Get_Coeff_in().size() < 7) {
                TeEDebug(">>: 初次圆柱拟合未获得有效结果，流程结束。");
                return;
            }
            ContinueFitCy3AfterInitialFit(fcy, Cloud_Temp);
        });
}

// 初次拟合完成后的界面部分: 可视化 + 收集优化/焊缝参数 + 启动后台优化
void CloudForgeAnalyzer::ContinueFitCy3AfterInitialFit(std::shared_ptr<Fit_Cylinder> fcy,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp)
{
    Eigen::VectorXf coeff1 = fcy->Get_Coeff_in();
    pcl::ModelCoefficients::Ptr cycoeff1(new pcl::ModelCoefficients);
    cycoeff1->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cycoeff1->values[i] = coeff1(i);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Inliers = fcy->Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Outliers = fcy->Get_Outliers();
    ColorManager color_inliers(0, 255, 0);   // 绿色-内点
    ColorManager color_outliers(255, 0, 0);  // 红色-外点
    beginUndoBatch("初始圆柱拟合");
    AddPointCloud("initial_fit_inliers", Cloud_Inliers, color_inliers);
    AddPointCloud("initial_fit_outliers", Cloud_Outliers, color_outliers);
    endUndoBatch();

    viewer->addCylinder(*cycoeff1, "initial_fit_cylinder");

    // 输出初次拟合信息
    Update_CFmes(fcy->message);
    TeEDebug(">>: 初次圆柱拟合完成。内点(绿)/外点(红)与圆柱体(initial_fit_cylinder)已可视化。");

    Eigen::Vector3f initial_center = fcy->get_center_point();
    Eigen::Vector3f initial_axis = fcy->get_axis_direction();

    // 3. 优化参数设置
    ParamDialogMeausreCy pdialog;
    if (pdialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消，优化跳过。初次拟合结果已保存。");
        addCylinderResult(GenerateRandomName("fitted_cylinder"), cycoeff1);
        return;
    }

    double design_radius = pdialog.getParams()[0].toDouble();
    double tolerance = pdialog.getParams()[1].toDouble();
    int iters = pdialog.getParams()[2].toInt();

    // 3.5 焊缝检测参数设置（取消则跳过焊缝约束，仅做无约束优化）
    bool useWeldConstraint = false;
    ParamDialogWeld wdialog;
    double weldThrFactor = 0.0, weldClusterTol = 0.0, weldLambda = 0.0;
    int weldMinCluster = 0;
    if (wdialog.exec() == QDialog::Accepted) {
        useWeldConstraint = true;
        weldThrFactor = wdialog.getParams()[0].toDouble();
        weldClusterTol = wdialog.getParams()[1].toDouble();
        weldMinCluster = wdialog.getParams()[2].toInt();
        weldLambda = wdialog.getParams()[3].toDouble();
    }
    else {
        TeEDebug(">>: 焊缝参数设置取消，跳过焊缝约束。");
    }

    // 4. 两阶段优化 + 焊缝约束修正（第三阶段）—— 放到后台线程执行
    auto evaluator = std::make_shared<MeasureCylindricity>();
    evaluator->setInputCloud(Cloud_Temp);
    evaluator->setDesignRadius(design_radius);
    evaluator->setTolerance(tolerance);
    evaluator->setMaxIterations(iters);
    evaluator->setVerbose(false);
    evaluator->setInitialLine(initial_center, initial_axis); // 设置初始值
    if (useWeldConstraint) {
        evaluator->setWeldThresholdFactor(weldThrFactor);
        evaluator->setWeldClusterTolerance(weldClusterTol);
        evaluator->setWeldMinClusterSize(weldMinCluster);
        evaluator->setWeldConstraintWeight(weldLambda);
    }
    evaluator->setProgressCallback(WorkerProgressCallback());

    auto resultPtr = std::make_shared<MeasureCylindricity::AssessmentResult>();
    RunAsyncVoid("圆柱轴线优化(含焊缝约束)",
        [evaluator, resultPtr, useWeldConstraint]() {
            *resultPtr = useWeldConstraint ? evaluator->evaluateCylindricityWithWeld()
                                           : evaluator->evaluateCylindricity();
        },
        [this, resultPtr, evaluator, initial_center, initial_axis, design_radius]() {
            if (evaluator->isCancelled()) {
                TeEDebug(">>: 圆柱轴线优化已取消，未保存结果。");
                return;
            }
            FinishFitCy3(resultPtr, evaluator, initial_center, initial_axis, design_radius);
        });
}

// 优化完成后的界面部分: 焊缝点/优化圆柱/轴线可视化 + 保存结果 + 报告
void CloudForgeAnalyzer::FinishFitCy3(const std::shared_ptr<MeasureCylindricity::AssessmentResult>& result,
                                      const std::shared_ptr<MeasureCylindricity>& evaluator,
                                      const Eigen::Vector3f& initial_center,
                                      const Eigen::Vector3f& initial_axis,
                                      double design_radius)
{
    // 5. 可视化检测到的焊缝点（橙色），供用户核对
    auto weld_cloud = evaluator->getWeldPoints();
    if (weld_cloud && !weld_cloud->empty()) {
        ColorManager color_weld(255, 128, 0);
        beginUndoBatch("焊缝点");
        AddPointCloud("weld_points", weld_cloud, color_weld);
        endUndoBatch();
        TeEDebug(">>: 检测到 " + std::to_string(weld_cloud->size()) + " 个焊缝点（橙色），已参与轴向修正。");
    }
    else {
        TeEDebug(">>: 未检测到显著焊缝带，结果为无约束优化。");
    }

    // 6. 优化后圆柱与轴线可视化
    Eigen::Vector3f optimized_center = result->getCylinderAxisPoint();
    Eigen::Vector3f optimized_axis = result->getCylinderAxisDirection();

    pcl::ModelCoefficients::Ptr cycoeff2(new pcl::ModelCoefficients);
    cycoeff2->values.resize(7);
    cycoeff2->values[0] = optimized_center.x();
    cycoeff2->values[1] = optimized_center.y();
    cycoeff2->values[2] = optimized_center.z();
    cycoeff2->values[3] = optimized_axis.x();
    cycoeff2->values[4] = optimized_axis.y();
    cycoeff2->values[5] = optimized_axis.z();
    cycoeff2->values[6] = static_cast<float>(design_radius); // 使用设定的设计半径

    viewer->addCylinder(*cycoeff2, "optimized_fit_cylinder");

    vtkSmartPointer<vtkLineSource> lineSource2 = vtkSmartPointer<vtkLineSource>::New();
    optimized_axis.normalize();
    float line_length = 800.0f;
    Eigen::Vector3f p1_opt = optimized_center - optimized_axis * line_length;
    Eigen::Vector3f p2_opt = optimized_center + optimized_axis * line_length;
    lineSource2->SetPoint1(p1_opt.x(), p1_opt.y(), p1_opt.z());
    lineSource2->SetPoint2(p2_opt.x(), p2_opt.y(), p2_opt.z());
    lineSource2->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper2->SetInputConnection(lineSource2->GetOutputPort());

    vtkSmartPointer<vtkActor> lineActor2 = vtkSmartPointer<vtkActor>::New();
    lineActor2->SetMapper(mapper2);
    lineActor2->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色轴线
    lineActor2->GetProperty()->SetLineWidth(3.0);

    AddActors(GenerateRandomName("axis"), lineActor2);

    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();

    // 7. 存储最终优化结果
    std::string storedName = GenerateRandomName("optimized_cylinder");
    addCylinderResult(storedName, cycoeff2);

    std::stringstream ss;
    ss << "圆柱拟合与焊缝约束优化完成。\n";
    ss << "初次拟合轴线点: (" << initial_center.x() << ", "
        << initial_center.y() << ", " << initial_center.z() << ")";
    ss << "初次拟合轴线方向: (" << initial_axis.x() << ", "
        << initial_axis.y() << ", " << initial_axis.z() << ")";
    ss << "优化后轴线点: (" << optimized_center.x() << ", "
        << optimized_center.y() << ", " << optimized_center.z() << ")";
    ss << "优化后轴线方向: (" << optimized_axis.x() << ", "
        << optimized_axis.y() << ", " << optimized_axis.z() << ")";
    std::string finalMsg = "圆柱拟合与焊缝约束优化完成。\n";
    finalMsg += "初次拟合：内点(绿)/外点(红)，圆柱体 'initial_fit_cylinder'\n";
    finalMsg += "优化：圆柱体 'optimized_fit_cylinder'，焊缝点(橙) 'weld_points'\n";

    TeEDebug(">>: 焊缝约束优化完成，几何体已更新。");
    Update_CFmes(finalMsg + ss.str());
}

void CloudForgeAnalyzer::Tool_MeasureArc() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择被测点云");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>:未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    ChoseCyDialog dialog1(cylinderResultsMap);
    if (dialog1.getSelectedList().empty()) {
        return;
    }
    pcl::ModelCoefficients::Ptr cy_Temp = cylinderResultsMap[dialog1.getSelectedList()[0]];

    qDebug() << "===== 圆柱参数验证 =====";
    qDebug() << QString("轴线点: (%1, %2, %3)")
        .arg(cy_Temp->values[0])
        .arg(cy_Temp->values[1])
        .arg(cy_Temp->values[2]);

    Eigen::Vector3f axis_dir(cy_Temp->values[3], cy_Temp->values[4], cy_Temp->values[5]);
    float norm = axis_dir.norm();
    qDebug() << "轴线方向向量模长:" << norm;

    if (norm < 0.001f) {
        TeEDebug("Error: 圆柱轴线方向向量模长过小，拟合可能有问题");
        return;
    }

    qDebug() << "圆柱半径:" << cy_Temp->values[6];
    qDebug() << "点云点数:" << Cloud_Temp->size();

	MeasureArc::FitMethod fitMethod = MeasureArc::BSPLINE_LSQ;
    // 手动拾取的测量位置点: 用 shared_ptr 承载, 供工作线程期间保持生命周期
    auto interestPointPtr = std::make_shared<pcl::PointXYZ>();
    bool hasInterestPoint = false;

    OptionBox box({ "Cardinal样条(插值)", "B样条逼近(平滑)" }, nullptr);
    box.setTitle("选择拟合方法");
    box.setMessage("请选择曲线拟合方法:");
    box.setDefaultOption(1); // 默认选中第一个选项

    if (box.exec() == QDialog::Accepted) {
        int index = box.getSelectedIndex();
        QString text = box.getSelectedText();

        if (index == 0) {
			fitMethod = MeasureArc::CARDINAL_SPLINE;
            qDebug() << "选择了Cardinal样条";
            TeEDebug("选择了Cardinal样条");
        }
        else if (index == 1) {
			fitMethod = MeasureArc::BSPLINE_LSQ;
            qDebug() << "选择了B样条逼近";
            TeEDebug("选择了B样条逼近");
        }
    }
    else {
        qDebug() << "操作取消";
    }

    
    OptionBox box1({ "测量中线", "手动选择" }, nullptr);
    box.setTitle("选择测量位置");
    box.setMessage("请选择选择测量位置:");
    box.setDefaultOption(0);
    if (box1.exec() == QDialog::Accepted) {
        int index = box1.getSelectedIndex();
        QString text = box1.getSelectedText();

        if (index == 0) {
            hasInterestPoint = false;
            qDebug() << "测量中线";
            TeEDebug("测量中线");
        }
        else if (index == 1) {

            qDebug() << "手动选择测量位置";
            TeEDebug("手动选择测量位置");
            TeEDebug("请选择高度");
            PointPickerMgr mgr(ui->winOfAnalyzer->interactor(), 1);
            auto pts_pcl = mgr.GetPickedPCLPoints();
            if (pts_pcl.size() < 1) {
                TeEDebug("点选择已取消或不足一个点");
                return;
            }
            *interestPointPtr = pts_pcl[0];//之前写法：interest_point = &pts_pcl[0]; mgr.GetPickedPCLPoints();返回的是临时变量，在此语块后被释放造成错误
            hasInterestPoint = true;
        }
    }
    else {
        qDebug() << "操作取消";
		TeEDebug("操作取消");
        return;
    }

    // 拟合参数对话框在界面线程弹出(原先在 MeasureArc 构造函数里弹框)
    MeasureArc::ArcParams arcParams;
    arcParams.method = fitMethod;
    if (fitMethod == MeasureArc::CARDINAL_SPLINE) {
        ParamDialogMeaArc arcDialog;
        if (arcDialog.exec() != QDialog::Accepted) {
            TeEDebug(">>: 参数设置取消");
            return;
        }
        bool ok1, ok2, ok3, ok4;
        arcParams.sliceThicknessFactor = arcDialog.getParams()[0].toDouble(&ok1);
        arcParams.integrationTolerance = arcDialog.getParams()[1].toDouble(&ok2);
        arcParams.downsampleTargetSize = arcDialog.getParams()[2].toInt(&ok3);
        arcParams.virtualPointExtrapolation = arcDialog.getParams()[3].toDouble(&ok4);
        if (!ok1 || !ok2 || !ok3 || !ok4) {
            TeEDebug(">>: 无效数字");
            return;
        }
    }
    else {
        ParamDialogMeaArcB arcDialog;
        if (arcDialog.exec() != QDialog::Accepted) {
            TeEDebug(">>: 参数设置取消");
            return;
        }
        bool ok1, ok2, ok3, ok4, ok5;
        arcParams.sliceThicknessFactor = arcDialog.getParams()[0].toDouble(&ok1);
        arcParams.integrationTolerance = arcDialog.getParams()[1].toDouble(&ok2);
        arcParams.bsplineDegree = arcDialog.getParams()[2].toInt(&ok3);
        arcParams.bsplineControlPoints = arcDialog.getParams()[3].toInt(&ok4);
        arcParams.bsplineSmoothingFactor = arcDialog.getParams()[4].toDouble(&ok5);
        if (!ok1 || !ok2 || !ok3 || !ok4 || !ok5) {
            TeEDebug(">>: 无效数字");
            return;
        }
    }

    // 后台执行弧长计算(切片投影+样条拟合+积分), 完成后回到界面线程添加曲线Actor
    auto measurerHolder = std::make_shared<std::shared_ptr<MeasureArc>>();
    RunAsyncVoid("圆弧测量",
        [this, measurerHolder, Cloud_Temp, cy_Temp, interestPointPtr, hasInterestPoint, arcParams]() {
            if (!PostProgress(0, 0, "圆弧测量: 切片投影与样条拟合...")) {
                PostLog(">>: 圆弧测量已取消。");
                return;
            }
            pcl::PointXYZ* interest_point = hasInterestPoint ? interestPointPtr.get() : nullptr;
            // 参数构造函数不再弹框、不再自动计算, 由工作线程显式调用 compute()
            *measurerHolder = std::make_shared<MeasureArc>(Cloud_Temp, cy_Temp, interest_point, arcParams);
            (*measurerHolder)->setAutoBuildActor(false);   // VTK Actor 改为在界面线程创建
            (*measurerHolder)->compute();
            const MeasureArc& m = **measurerHolder;
            if (m.success) {
                PostLog("截面弧长: " + std::to_string(m.arcLength) + " mm");
            }
            else {
                PostLog(">>: 弧长计算失败: " + m.message);
            }
            PostProgress(100, 100, "圆弧测量完成");
        },
        [this, measurerHolder]() {
    if (!*measurerHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 圆弧测量已取消，不再显示本次结果。");
        return;
    }
    MeasureArc& measurer = **measurerHolder;
    if (measurer.success && (!measurer.isCancelled)) {
        // 1. 获取计算生成的曲线Actor（VTK 对象在界面线程创建）
        measurer.buildVisualizationActor();
        vtkSmartPointer<vtkActor> splineActor = measurer.getVisualizationActor();

        if (splineActor) {
            // 2. 生成一个唯一的ID用于管理
            std::string actorId = GenerateRandomName("arc_spline_");

            AddActors(actorId, splineActor);

            TeEDebug(">>: 弧长曲线已成功添加到3D视图。");

            std::string resultMsg = "截面弧长: " + std::to_string(measurer.arcLength) + " mm";
            Update_CFmes(resultMsg);
        }
        else {
            TeEDebug(">>: 警告：未能获取到有效的曲线可视化对象。");
        }
    }
    else {
        TeEDebug(">>: 弧长计算失败，无法生成可视化曲线。");
    }

    TeEDebug(measurer.message);
        });
}


void CloudForgeAnalyzer::visualizeCylindricityHeatMap(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud,
    double min_distance, double max_distance)
{
	TeEDebug("开始可视化圆柱度热力图...");
    if (!heatmap_cloud || heatmap_cloud->empty()) {
        TeEDebug("热力图点云为空，无法可视化");
        return;
    }

    // 先清除可能存在的旧热力图及标注
    viewer->removePointCloud("cylindricity_heatmap");
    viewer->removeShape("heatmap_colorbar"); // 清除旧的颜色条（如果存在）
    viewer->removeText3D("heatmap_title");

    // 添加热力图点云
    viewer->addPointCloud<pcl::PointXYZRGB>(heatmap_cloud, "cylindricity_heatmap");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylindricity_heatmap");
    // === 新增：创建并添加固定颜色条 (Colorbar) ===
    // 1. 获取当前渲染器
    RGBCloudMap.emplace("cylindricity_heatmap",heatmap_cloud);
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        TeEDebug("错误：无法获取渲染器，颜色条创建失败。");
    }
    else {
        // 2. 创建颜色查找表 (Lookup Table)，映射从绿到红
        vtkNew<vtkLookupTable> hueLut;
        hueLut->SetTableRange(min_distance, max_distance); // 标量值范围对应误差范围
        hueLut->SetHueRange(0.33, 0.0);      // 从绿色 (0.33) 到红色 (0.0)
        hueLut->SetSaturationRange(1, 1);
        hueLut->SetValueRange(1, 1);
        hueLut->SetNanColor(1, 1, 1, 1);    // 无效值显示为白色
        hueLut->SetNumberOfTableValues(256); // 颜色精度
        hueLut->Build();

        // 3. 创建颜色条 Actor
        vtkNew<vtkScalarBarActor> scalarBar;
        scalarBar->SetLookupTable(hueLut);
        scalarBar->SetTitle("mm");
        scalarBar->SetNumberOfLabels(5); // 主标签数量
        scalarBar->SetMaximumNumberOfColors(256);


        const int titleFontSize = 8;        // 颜色条标题文字大小
        const int labelFontSize = 10;        // 颜色条标签文字大小
        const double colorbarWidth = 0.05;   // 颜色条宽度 (占窗口宽度的比例，建议0.03-0.07)
        const double colorbarHeight = 0.6;   // 颜色条高度 (占窗口高度的比例，建议0.4-0.7)
        const double colorbarPosX = 0.92;    // 颜色条右侧位置 (范围0~1, 1为右边缘)
        const double colorbarPosY = 0.2;     // 颜色条底部位置 (范围0~1, 1为上边缘)

        // 设置颜色条文本属性（字号、颜色）
        vtkNew<vtkTextProperty> titleProperty;
        titleProperty->SetFontSize(titleFontSize);
        titleProperty->BoldOn();
        titleProperty->SetColor(1, 1, 1); // 白色
        scalarBar->SetTitleTextProperty(titleProperty);

        scalarBar->SetTitleRatio(0.6); // 增加标题区域所占的比例，例如从0.5调到0.6

        vtkNew<vtkTextProperty> labelProperty;
        labelProperty->SetFontSize(labelFontSize);
        labelProperty->SetColor(0.9, 0.9, 0.9); // 浅灰色
        scalarBar->SetLabelTextProperty(labelProperty);
        // === 变量定义结束 ===

        // 应用调整后的位置和大小
        scalarBar->SetPosition(colorbarPosX - colorbarWidth, colorbarPosY);
        scalarBar->SetWidth(colorbarWidth);
        scalarBar->SetHeight(colorbarHeight);


        scalarBar->SetPickable(0); // 禁止拾取，避免与点云交互冲突

        // 4. 将颜色条添加到渲染器，并赋予唯一名称以便管理
        renderer->AddActor2D(scalarBar);
        // 为了方便后续管理（如清除），可以将其存储在某个容器，但此处为最小修改，仅作添加。
        // 注意：PCLVisualizer 的 removeShape 可能无法管理此Actor，需单独处理。
    }
    // === 颜色条添加结束 ===

    TeEDebug("热力图与颜色条可视化完成");
    TeEDebug("误差范围: " + std::to_string(min_distance) + " - " + std::to_string(max_distance) + " mm");

    // 刷新渲染窗口
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

void CloudForgeAnalyzer::Tool_MeasureCylindricity()
{
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // 1. 选择待评估的点云
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择待评估圆柱度的点云");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = CloudMap[dialog.getSelectedList()[0]];

    // 2. 选择已有的圆柱拟合结果作为评估基准
    ChoseCyDialog dialog1(cylinderResultsMap);
    if (dialog1.getSelectedList().empty()) {
        TeEDebug(">>: 未选择圆柱拟合结果");
        return;
    }

    std::string selectedCylinderName = dialog1.getSelectedList()[0];
    pcl::ModelCoefficients::Ptr selected_cylinder = cylinderResultsMap[selectedCylinderName];
    std::string visualization_id = "measurement_ref_" + selectedCylinderName;
    viewer->addCylinder(*selected_cylinder, visualization_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        0.0, 1.0, 1.0, // 青色，以示区别
        visualization_id);
    TeEDebug(">>: 参考圆柱几何体 '" + selectedCylinderName + "' 已可视化(青色)。");

    // 3. 从选择的圆柱结果中提取轴线参数
    std::vector<float> axis_coeffs = {
        selected_cylinder->values[0], // center.x
        selected_cylinder->values[1], // center.y
        selected_cylinder->values[2], // center.z
        selected_cylinder->values[3], // axis.x
        selected_cylinder->values[4], // axis.y
        selected_cylinder->values[5]  // axis.z
    };
    double cylinder_design_radius = selected_cylinder->values[6];

    // 4. 获取评估参数（容差、迭代次数等）
    ParamDialogMeausreCy pdialog;
    if (pdialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    double tolerance = pdialog.getParams()[1].toDouble();
    int iters = pdialog.getParams()[2].toInt(); // 此处的迭代次数对直接评估影响有限，可保留

    // 5. 创建评估器并设置参数(计算在后台线程执行, 界面保持响应)
    auto evaluator = std::make_shared<MeasureCylindricity>();
    evaluator->setInputCloud(target_cloud);
    evaluator->setDesignRadius(cylinder_design_radius);
    evaluator->setTolerance(tolerance);
    evaluator->setMaxIterations(iters);
    evaluator->setVerbose(false);
    evaluator->setProgressCallback(WorkerProgressCallback());

    // 关键步骤：直接传入圆柱轴线参数，跳过优化阶段的参数寻优
    evaluator->setInitialLineFromCoeffs(axis_coeffs);

    // 6. 后台执行评估（该函数不进行优化，仅基于给定直线评估）
    auto resultPtr = std::make_shared<MeasureCylindricity::AssessmentResult>();
    RunAsyncVoid("圆柱度评估",
        [this, evaluator, resultPtr, axis_coeffs]() {
            PostProgress(0, 0, "圆柱度评估: 基于给定轴线计算偏差...");
            Eigen::Vector3f center(axis_coeffs[0], axis_coeffs[1], axis_coeffs[2]);
            Eigen::Vector3f axis(axis_coeffs[3], axis_coeffs[4], axis_coeffs[5]);
            *resultPtr = evaluator->evaluateGivenLine(center, axis); // 使用此函数直接评估，不优化
            PostLog(resultPtr->assessment_message);
        },
        [this, evaluator, resultPtr]() {
    if (evaluator->isCancelled()) {
        TeEDebug(">>: 圆柱度评估已取消，未保存结果。");
        return;
    }
    const auto& result = *resultPtr;

    // 7. 生成并可视化热力图 (即使不优化，也需要基于给定直线生成热力图)
    auto heatmap_cloud = evaluator->getHeatMapCloud();
    double min_distance, max_distance;
    evaluator->getDistanceRange(min_distance, max_distance);

    visualizeCylindricityHeatMap(heatmap_cloud, min_distance, max_distance);

    // 8. 显示评估结果
    qDebug() << result.assessment_message;
    TeEDebug(result.assessment_message);
    Update_CFmes(result.assessment_message);

    // 9. 刷新视图
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
        });
}

void CloudForgeAnalyzer::Tool_MeasureWeldHeight()
{
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // === 1. 选择焊缝点云（支持勾选多个）===
    ChoseCloudDialog dialogWeld(CloudMap, ColorMap, "选择焊缝点云（可勾选多个）");
    if (dialogWeld.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    auto weldNames = dialogWeld.getSelectedList();
    if (weldNames.empty()) {
        TeEDebug(">>: 未选择焊缝点云");
        return;
    }

    // === 2. 选择壁面点云（单选）===
    ChoseCloudDialog dialogBase(CloudMap, ColorMap, "选择圆柱壁面点云");
    if (dialogBase.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialogBase.getSelectedList().empty()) {
        TeEDebug(">>: 未选择壁面点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud = CloudMap[dialogBase.getSelectedList()[0]];

    // === 3. 参数设置 ===
    ParamDialogMeasureWeldHeight paramDialog;
    if (paramDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    auto params = paramDialog.getParams();
    bool ok1, ok2, ok3, ok4;
    double searchR  = params[0].toDouble(&ok1);
    double regionSz = params[1].toDouble(&ok2);
    double ransacTh = params[2].toDouble(&ok3);
    int minNei      = params[3].toInt(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4 || searchR <= 0 || regionSz <= 0) {
        TeEDebug(">>: 参数无效");
        return;
    }

    // === 4. 收集待处理的焊缝点云（保持原有顺序与下标，命名与原先一致）===
    std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> weldClouds;
    weldClouds.reserve(weldNames.size());
    for (const auto& weldName : weldNames) {
        weldClouds.emplace_back(weldName, CloudMap[weldName]);
    }

    // 每条焊缝的后台计算结果(评估结果 + 热力图 + 高度范围), 由 worker 与 onFinished 共享
    struct WeldOutcome {
        std::string weldName;
        std::string prefix;
        MeasureWeldHeight::AssessmentResult result;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap;
        double minH = 0.0;
        double maxH = 0.0;
        double maxAbsH = 0.0;
    };
    auto outcomes = std::make_shared<std::vector<WeldOutcome>>();

    // === 5. 后台执行测量（界面保持响应；可在工具栏“取消计算”中止）===
    RunAsyncVoid("焊缝高度测量",
        [this, outcomes, weldClouds, baseCloud, searchR, regionSz, ransacTh, minNei]() {
            const int total = static_cast<int>(weldClouds.size());
            for (int wi = 0; wi < total; ++wi) {
                const std::string& weldName = weldClouds[wi].first;
                pcl::PointCloud<pcl::PointXYZ>::Ptr weldCloud = weldClouds[wi].second;
                // 进度回报: 返回 false 表示用户点击了“取消计算”
                if (!PostProgress(wi, total, "焊缝高度测量: " + weldName)) {
                    PostLog(">>: 焊缝高度测量已取消。");
                    break;
                }
                if (!weldCloud || weldCloud->empty()) {
                    PostLog(">>: 焊缝点云 '" + weldName + "' 为空，跳过");
                    continue;
                }
                PostLog(">>: 正在评估焊缝 '" + weldName + "' ...");

                WeldOutcome outcome;
                outcome.weldName = weldName;
                outcome.prefix = "weld_" + std::to_string(wi) + "_";

                MeasureWeldHeight measurer;
                measurer.setWeldCloud(weldCloud);
                measurer.setBaseCloud(baseCloud);
                measurer.setSearchRadius(searchR);
                measurer.setRegionSize(regionSz);
                measurer.setRansacThreshold(ransacTh);
                measurer.setMinNeighbors(minNei);
                measurer.setVerbose(false);

                outcome.result = measurer.evaluate();
                outcome.heatmap = measurer.getHeatMapCloud();
                measurer.getHeightRange(outcome.minH, outcome.maxH);
                outcome.maxAbsH = measurer.getMaxAbsHeight();

                PostLog(outcome.result.assessment_message);
                outcomes->push_back(outcome);
            }
            PostProgress(total, total, "焊缝高度测量完成");
        },
        [this, outcomes, regionSz]() {
    // === 6. 可视化与报告（仅在 GUI 线程执行）===
    m_weldMeasureShapeIds.clear();
    std::string allReports;
    for (const auto& outcome : *outcomes) {
        const std::string& weldName = outcome.weldName;
        const std::string& prefix = outcome.prefix;
        const auto& result = outcome.result;
        auto heatmap = outcome.heatmap;
        double minH = outcome.minH;
        double maxH = outcome.maxH;
        double maxAbsH = outcome.maxAbsH;

        // 追踪形状 ID，供撤销时清理
        m_weldMeasureShapeIds.push_back(prefix + "heatmap");
        m_weldMeasureShapeIds.push_back(prefix + "sphere_high");
        m_weldMeasureShapeIds.push_back(prefix + "text_high");

        if (heatmap && !heatmap->empty()) {
            std::string heatmap_id = prefix + "heatmap";

            // 清除旧的
            viewer->removePointCloud(heatmap_id);
            viewer->removeShape(prefix + "colorbar");
            viewer->removeShape(prefix + "sphere_high");
            viewer->removeText3D(prefix + "text_high");

            // 添加热力图点云
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(heatmap);
            viewer->addPointCloud<pcl::PointXYZRGB>(heatmap, rgb, heatmap_id);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, heatmap_id);
            RGBCloudMap.emplace(heatmap_id, heatmap);

            // --- 颜色条（双方向发散）---
            vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
            if (renderer) {
                // 先清除所有旧的 vtkScalarBarActor，避免累积
                {
                    vtkPropCollection* props = renderer->GetViewProps();
                    props->InitTraversal();
                    std::vector<vtkProp*> barsToRemove;
                    vtkProp* prop;
                    while ((prop = props->GetNextProp()) != nullptr) {
                        if (vtkScalarBarActor::SafeDownCast(prop)) {
                            barsToRemove.push_back(prop);
                        }
                    }
                    for (auto p : barsToRemove) {
                        renderer->RemoveActor2D(static_cast<vtkActor2D*>(p));
                    }
                }

                // 用 [0, 1] 正范围构建发散色表（规避 VTK 负值范围标签 bug）
                vtkNew<vtkLookupTable> lut;
                lut->SetTableRange(0.0, 1.0);
                lut->SetNumberOfTableValues(256);

                for (int i = 0; i < 256; ++i) {
                    double t = i / 255.0;  // 0(蓝) → 0.5(绿) → 1(红)
                    double r, g, b;
                    if (t < 0.5) {
                        double s = t * 2;
                        r = 0.0; g = s; b = 1.0 - s;
                    } else {
                        double s = (t - 0.5) * 2;
                        r = s; g = 1.0 - s; b = 0.0;
                    }
                    lut->SetTableValue(i, r, g, b);
                }

                // 设置标注：只在三个关键位置显示文字
                auto fmtVal = [](double v) -> std::string {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%.1f", v);
                    return std::string(buf);
                };
                lut->SetAnnotation(vtkVariant(0.0), fmtVal(-maxAbsH));
                lut->SetAnnotation(vtkVariant(0.5), "0.0");
                lut->SetAnnotation(vtkVariant(1.0), fmtVal(maxAbsH));
                lut->Build();

                vtkNew<vtkScalarBarActor> scalarBar;
                scalarBar->SetLookupTable(lut);
                scalarBar->SetTitle("mm");
                scalarBar->SetMaximumNumberOfColors(256);
                scalarBar->SetTextPosition(vtkScalarBarActor::PrecedeScalarBar);
                scalarBar->SetDrawTickLabels(0);   // 关闭自动刻度标签，仅显示 annotations

                vtkNew<vtkTextProperty> titleProp;
                titleProp->SetFontSize(8);
                titleProp->BoldOn();
                titleProp->SetColor(1, 1, 1);
                scalarBar->SetTitleTextProperty(titleProp);

                vtkNew<vtkTextProperty> labelProp;
                labelProp->SetFontSize(8);
                labelProp->SetColor(0.9, 0.9, 0.9);
                scalarBar->SetLabelTextProperty(labelProp);

                scalarBar->SetPosition(0.87, 0.2);
                scalarBar->SetWidth(0.06);
                scalarBar->SetHeight(0.6);
                scalarBar->SetPickable(0);
                renderer->AddActor2D(scalarBar);
            }

            // --- 4b. 最高点标注 ---
            if (result.valid_points > 0) {
                std::string sphere_id = prefix + "sphere_high";
                std::string text_id   = prefix + "text_high";

                viewer->removeShape(sphere_id);
                viewer->removeText3D(text_id);

                const auto& hp = result.highest_point;
                viewer->addSphere(hp, regionSz * 0.3, 1.0, 0.84, 0.0, sphere_id);
                viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.84, 0.0, sphere_id);
                viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, sphere_id);

                // 3D 文字标签
                std::string labelText = "Max: " + std::to_string(result.highest_value).substr(0, 6) + " mm";
                viewer->addText3D(labelText,
                    pcl::PointXYZ(hp.x + regionSz * 0.5, hp.y, hp.z + regionSz * 0.5),
                    0.5, 1.0, 1.0, 1.0, text_id);
            }

            TeEDebug(">>: 焊缝 '" + weldName + "' 热力图与最高点标注完成");
        }

        // --- 4c. 输出报告 ---
        TeEDebug(result.assessment_message);
        if (!allReports.empty()) allReports += "\n\n";
        allReports += result.assessment_message;
    }
    Update_CFmes(allReports);

    // === 5. 刷新视图 ===
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    TeEDebug(">>: 焊缝高度测量完成。");
        });
}

void CloudForgeAnalyzer::Tool_MeasurePothole()
{
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // 1. 选择待测点云（含凹塘/凹坑的多片拼接区域）
    ChoseCloudDialog dialog(CloudMap, ColorMap, "选择待测量凹塘的点云（建议为多片拼接区域）");
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = CloudMap[dialog.getSelectedList()[0]];

    // 2. 选择已保存的圆柱拟合结果作为理想柱面（与圆柱度测量一致：先拟合、后测量）
    //    注意: 基础"拟合圆柱"不保存结果，请使用"二次优化圆柱"或"焊缝约束优化"入口
    ChoseCyDialog dialog1(cylinderResultsMap);
    if (dialog1.getSelectedList().empty()) {
        TeEDebug(">>: 未选择圆柱拟合结果");
        return;
    }

    std::string selectedCylinderName = dialog1.getSelectedList()[0];
    pcl::ModelCoefficients::Ptr selected_cylinder = cylinderResultsMap[selectedCylinderName];

    // 参考圆柱可视化（青色，以示区别）
    std::string visualization_id = "pothole_ref_" + selectedCylinderName;
    viewer->removeShape(visualization_id);
    viewer->addCylinder(*selected_cylinder, visualization_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        0.0, 1.0, 1.0, visualization_id);
    TeEDebug(">>: 参考圆柱几何体 '" + selectedCylinderName + "' 已可视化(青色)。");

    // 3. 测量参数（阈值/容差为0时自动）
    ParamDialog_Pothole pdialog;
    if (pdialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 操作取消");
        return;
    }
    bool ok1, ok2, ok3;
    double thr = pdialog.getParams()[0].toDouble(&ok1);
    double ctol = pdialog.getParams()[1].toDouble(&ok2);
    int minpts = pdialog.getParams()[2].toInt(&ok3);
    if (!ok1 || !ok2 || !ok3 || thr < 0 || ctol < 0 || minpts <= 0) {
        TeEDebug(">>: 无效参数");
        return;
    }
    // 形面趋势处理模式与点群面积占比上限(新增)
    const QString trendStr = pdialog.getParams().size() > 3 ? pdialog.getParams()[3] : QString("0");
    const QString areaStr = pdialog.getParams().size() > 4 ? pdialog.getParams()[4] : QString("20");
    bool okT = true, okA = true;
    int trendMode = trendStr.toInt(&okT);
    double areaPct = areaStr.toDouble(&okA);
    if (!okT || trendMode < 0 || trendMode > 2) trendMode = 0;
    if (!okA || areaPct <= 0 || areaPct > 100) areaPct = 20.0;

    // 局部基准(口径B)参数: 窗口 W / 局部阈值 T_local / 是否启用 / 热力图显示场(新增, 追加在末尾)
    const QString winStr = pdialog.getParams().size() > 5 ? pdialog.getParams()[5] : QString("90");
    const QString lthrStr = pdialog.getParams().size() > 6 ? pdialog.getParams()[6] : QString("0.35");
    const QString useLocalStr = pdialog.getParams().size() > 7 ? pdialog.getParams()[7] : QString("1");
    const QString heatStr = pdialog.getParams().size() > 8 ? pdialog.getParams()[8] : QString("0");
    bool okW = true, okL = true, okU = true, okH = true;
    double localWindow = winStr.toDouble(&okW);
    double localThr = lthrStr.toDouble(&okL);
    int useLocal = useLocalStr.toInt(&okU);
    int heatField = heatStr.toInt(&okH);
    if (!okW || localWindow <= 0) localWindow = 90.0;
    if (!okL || localThr < 0) localThr = 0.35;
    if (!okU || (useLocal != 0 && useLocal != 1)) useLocal = 1;
    if (!okH || (heatField != 0 && heatField != 1)) heatField = 0;

    // 4. 后台执行测量（界面保持响应；可在工具栏“取消计算”中止）
    auto potholePtr = std::make_shared<MeasurePothole>();
    auto pitResult = std::make_shared<MeasurePothole::PitResult>();
    RunAsyncVoid("凹塘测量",
        [this, potholePtr, pitResult, target_cloud, selected_cylinder, thr, ctol, minpts,
         trendMode, areaPct, localWindow, localThr, useLocal, heatField]() {
            PostProgress(0, 0, "凹塘测量: 残差与点群聚类...");
            potholePtr->setInputCloud(target_cloud);
            potholePtr->setCylinder(selected_cylinder);
            potholePtr->setDistanceThreshold(thr);
            potholePtr->setClusterTolerance(ctol);
            potholePtr->setMinClusterSize(minpts);
            potholePtr->setTrendMode(trendMode);
            potholePtr->setMaxAreaFraction(areaPct / 100.0);
            potholePtr->setUseLocalBaseline(useLocal != 0);
            potholePtr->setLocalWindow(localWindow);
            potholePtr->setLocalThreshold(localThr);
            potholePtr->setBoundaryExclude(true);
            potholePtr->setHeatMapField(heatField);
            potholePtr->setVerbose(false);
            *pitResult = potholePtr->evaluate();
            // 调试框只留一行结论: 个数/主坑深度/全局最大距离/判定
            {
                int okCount = 0;
                for (const auto& it : pitResult->pits) if (it.valid) ++okCount;
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3);
                oss << ">> [凹塘] 检出 " << pitResult->pit_count << " 个（通过 " << okCount
                    << "） | 主坑深 " << pitResult->local_max_depth
                    << " mm | 到理想柱面最大距离 " << pitResult->max_depth
                    << " mm | W=" << pitResult->local_window << " mm"
                    << " | " << (pitResult->valid ? "检出有效凹塘" : "未判定为凹塘");
                PostLog(oss.str());
            }
        },
        [this, potholePtr, pitResult]() {
            const auto& result = *pitResult;

    // 5. 残差热力图（蓝=凹，白=0，红=凸）
    auto heatmap_cloud = potholePtr->getHeatMapCloud();
    if (heatmap_cloud && !heatmap_cloud->empty()) {
        viewer->removePointCloud("pothole_heatmap");
        viewer->addPointCloud<pcl::PointXYZRGB>(heatmap_cloud, "pothole_heatmap");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pothole_heatmap");
        RGBCloudMap.erase("pothole_heatmap");
        RGBCloudMap.emplace("pothole_heatmap", heatmap_cloud);
        TeEDebug(std::string(">>: 热力图已显示（蓝=凹，红=凸）；当前显示 ")
            + (potholePtr->getHeatMapFieldUsed() == 0 ? "局部凹陷深度（相对局部基准）"
                                                      : "到理想柱面距离"));
    }

    // 6. 凹塘点群（橙色）
    auto pit_cloud = potholePtr->getPitCloud();
    if (pit_cloud && !pit_cloud->empty()) {
        ColorManager color_pit(255, 128, 0);
        beginUndoBatch("凹塘点群");
        AddPointCloud("pothole_pit_cloud", pit_cloud, color_pit);
        endUndoBatch();
        TeEDebug(">>: 凹塘点群已显示（橙色）。");
    }

    // 6.5 场景尺度: 3D 文字与标记球按补丁大小自适应
    //     旧实现 textScale 固定 0.5: 在 300 mm 级补丁上不足 2 个像素高, 实际看不见;
    //     这里取包围盒对角线的 2.2%(6~14 mm 兜底), 保证标签在整幅视图下清晰可读。
    double textScale = 10.0;   // 3D 文字高度(mm)
    double markScale = 4.0;    // 标记球基础半径(mm)
    {
        pcl::PointXYZRGB lo, hi;   // 热力图是 XYZRGB, 包围盒类型需与点云一致
        if (heatmap_cloud && !heatmap_cloud->empty()) {
            pcl::getMinMax3D(*heatmap_cloud, lo, hi);
            const double diag = (hi.getVector3fMap() - lo.getVector3fMap()).norm();
            textScale = std::min(std::max(0.022 * diag, 6.0), 14.0);
            // 球只是"定位点", 取 0.6% 对角线(约 φ6 mm): 再大就会把小坑整个盖住(φ11 坑短轴仅 6 mm)
            markScale = std::min(std::max(0.006 * diag, 2.0), 4.5);
        }
    }

    // 7. 最深点标注（球 + 3D文字）
    if (result.fit_ok) {
        const auto& dp = result.deepest_point;
        std::string sphere_id = "pothole_sphere_deepest";
        std::string text_id = "pothole_text_deepest";
        viewer->removeShape(sphere_id);
        viewer->removeText3D(text_id);

        double marker_size = std::max(markScale, result.max_depth * 0.5);
        viewer->addSphere(dp, marker_size, 0.0, 1.0, 1.0, sphere_id);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, sphere_id);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, sphere_id);

        std::stringstream depthSS;
        depthSS << std::fixed << std::setprecision(3) << result.max_depth;
        std::string labelText = "MAX dist " + depthSS.str() + " mm";
        // 这个点常常就在主坑最深处旁边(实测只差几个毫米), 若与各坑标签同高必然叠字。
        // 所以: 各坑标签抬 1.15 倍字高, 本条抬 2.6 倍字高(二者错开约 1.5 倍字高),
        // 并画一条引线连回球心, 保证"哪句话说的是哪个点"一目了然。
        const pcl::PointXYZ maxLabelPos(dp.x, dp.y, dp.z + marker_size + 2.6 * textScale);
        const std::string line_id = "pothole_line_deepest";
        viewer->removeShape(line_id);
        viewer->addLine<pcl::PointXYZ>(dp, maxLabelPos, 0.1, 1.0, 1.0, line_id);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, line_id);
        viewer->addText3D(labelText, maxLabelPos,
            textScale, 0.1, 1.0, 1.0, text_id);
    }

    // 7b. 多凹坑逐个标注(每个坑一个球 + 编号文字): 主坑洋红, 其余橙色;
    //     校验未通过的坑用灰白色球提示"该坑不判定"(仍给出编号与深度, 便于人工复核).
    //     PitItem::index 与报告中的坑编号一一对应.
    if (!result.pits.empty()) {
        for (int i = 0; i < static_cast<int>(result.pits.size()); ++i) {
            const auto& it = result.pits[i];
            if (!std::isfinite(it.centroid.x)) continue;
            const std::string tag = std::to_string(it.index);
            const std::string sphere_id = "pothole_pit_sphere_" + tag;
            const std::string text_id = "pothole_pit_text_" + tag;
            viewer->removeShape(sphere_id);
            viewer->removeText3D(text_id);

            // 标注位置: 优先用该坑最深点, 缺失时退化到质心
            pcl::PointXYZ mk = std::isfinite(it.deepest_point.x) ? it.deepest_point : it.centroid;
            const double depthForSize = it.judged_by_local ? it.local_max_depth : it.global_max_depth;
            const double marker_size = std::max(markScale, depthForSize * 0.5);

            // 球体颜色 = 状态; 文字颜色单独取高对比度(未通过的球是浅灰, 文字必须用深灰才看得清)
            double cr = 1.0, cg = 1.0, cb = 1.0;   // 未通过校验: 灰白
            double tr = 0.30, tg = 0.30, tb = 0.30;
            if (it.valid && it.is_main) { cr = 1.0; cg = 0.0; cb = 1.0; tr = 1.0; tg = 0.15; tb = 1.0; }
            else if (it.valid) { cr = 1.0; cg = 0.65; cb = 0.0; tr = 1.0; tg = 0.62; tb = 0.0; }
            viewer->addSphere(mk, marker_size, cr, cg, cb, sphere_id);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, cr, cg, cb, sphere_id);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, sphere_id);

            // 文字: 只保留"编号 + 主坑标记 + 深度(2 位小数)", 放大到 textScale
            std::stringstream pitSS;
            pitSS << std::fixed << std::setprecision(2);
            pitSS << "PIT " << it.index << (it.is_main ? " (MAIN)" : "")
                  << "  " << depthForSize << " mm" << (it.valid ? "" : "  reject");
            viewer->addText3D(pitSS.str(),
                pcl::PointXYZ(mk.x, mk.y, mk.z + marker_size + 1.15 * textScale),
                textScale, tr, tg, tb, text_id);
        }
        {
            std::string msg = ">>: 已逐个标注 " + std::to_string(result.pits.size())
                + " 个凹塘（文字=编号+深度，字高 "
                + std::to_string(static_cast<int>(textScale + 0.5))
                + " mm；球色区分主坑/其余/未通过）";
            TeEDebug(msg);
        }
    }

    // 8. 椭圆可视化（闭合折线，已回投到柱面）: 逐个绘制，仅绘制校验通过的坑；
    //    主坑青色，其余黄色(与球的橙色区分)。
    //    校验不通过的坑不画椭圆: 画出会把参考面偏差或边界缺失误示为凹坑。
    {
        int drawn = 0, skipped = 0;
        for (const auto& it : result.pits) {
            if (it.ellipse_points.empty()) { ++skipped; continue; }
            if (!it.valid) { ++skipped; continue; }

            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
            std::vector<vtkIdType> ids;
            ids.reserve(it.ellipse_points.size());
            for (const auto& p : it.ellipse_points) {
                ids.push_back(points->InsertNextPoint(p.x(), p.y(), p.z()));
            }
            lines->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());

            vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
            poly->SetPoints(points);
            poly->SetLines(lines);

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(poly);

            vtkSmartPointer<vtkActor> ellipseActor = vtkSmartPointer<vtkActor>::New();
            ellipseActor->SetMapper(mapper);
            if (it.is_main) ellipseActor->GetProperty()->SetColor(0.0, 1.0, 1.0);   // 主坑: 青色
            else            ellipseActor->GetProperty()->SetColor(1.0, 1.0, 0.0);   // 其余: 黄色
            ellipseActor->GetProperty()->SetLineWidth(3.0);

            AddActors(GenerateRandomName("pothole_ellipse_" + std::to_string(it.index)),
                ellipseActor);
            ++drawn;
        }
        {
            std::string msg = ">>: 凹塘轮廓椭圆已显示 " + std::to_string(drawn)
                + " 个（主坑青色 / 其余黄色）";
            if (skipped > 0) msg += "，另有 " + std::to_string(skipped) + " 个未绘制（未通过校验或椭圆拟合失败）";
            TeEDebug(msg);
        }
    }

    // 9. 输出报告并刷新视图
    Update_CFmes(result.assessment_message);
    TeEDebug(">>: 凹塘测量完成。");
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
        });
}

// ============================================================
// 焊前装配阶差(1.3) / 焊前装配间隙(1.4)
// 依据: docs/当前需新增功能/焊前装配阶差与间隙-独立执行方案.md §6/§7
//
// 操作路径(§7): 选两件 → 定接缝区域/测量段 → 确认近远角色(Z 均值) → 设 R 与测量方向
//              → 后台计算 → 检查边缘及极值。
// 约束: 计算只在 worker 线程; Qt/VTK actor/点云表/撤销只在 GUI 线程;
//       结果用 std::shared_ptr 回传(沿用 RunAsyncVoid / WorkerProgressCallback)。
// ============================================================
void CloudForgeAnalyzer::Tool_MeasureWeldPreparation()
{
    // 口径(2026-09 统一): 阶差 = 径向(近件在外为正); 间隙 = 沿跨缝方向(环缝=轴向)。
    // 两个量由同一次配对同时得到, 因此只有一个入口、一次计算、一份报告。
    const std::string metricName = "焊前装配测量";

    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    if (CloudMap.size() < 2) {
        TeEDebug(">>: 焊前测量需要两份点云(近件与远件); 当前点云数量不足两份。");
        return;
    }

    // ---- 1. 选择已保存的圆柱基准(轴线 + 设计半径) ----
    // 本功能不做任何圆柱拟合: 基准由"圆柱拟合/圆柱度"功能产生并全局保存, 这里只选它。
    if (cylinderResultsMap.empty()) {
        TeEDebug(">>: 还没有可用的圆柱基准。请先用圆柱拟合/圆柱度功能拟合远件并保存结果，再回到本功能选择它。");
        return;
    }
    // 注意: ChoseCyDialog 在构造函数内部已经 exec(), 这里不能再 exec()(否则会要求选两次)
    ChoseCyDialog dCy(cylinderResultsMap);
    if (dCy.getSelectedList().empty()) { TeEDebug(">>: 未选择圆柱基准, 操作取消"); return; }
    pcl::ModelCoefficients::Ptr cyl = cylinderResultsMap[dCy.getSelectedList()[0]];
    if (!cyl || cyl->values.size() < 7) {
        TeEDebug(">>: 所选圆柱结果缺少轴线或设计半径，无法作为基准。");
        return;
    }
    Eigen::Vector3d axisPoint(cyl->values[0], cyl->values[1], cyl->values[2]);
    Eigen::Vector3d axisDir(cyl->values[3], cyl->values[4], cyl->values[5]);
    const double designR = cyl->values[6];
    if (!axisDir.allFinite() || axisDir.norm() < 1e-9 || !(designR > 0.0)) {
        TeEDebug(">>: 所选圆柱结果的轴线或设计半径无效。");
        return;
    }
    axisDir.normalize();

    // ---- 2. 选择远件试样 ----
    ChoseCloudDialog dFar(CloudMap, ColorMap, "焊前装配测量: 选择【远件】试样点云");
    if (dFar.exec() != QDialog::Accepted) { TeEDebug(">>: 操作取消"); return; }
    if (dFar.getSelectedList().empty()) { TeEDebug(">>: 未选择远件点云"); return; }
    const std::string farName = dFar.getSelectedList()[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr farCloud = CloudMap[farName];
    if (!farCloud || farCloud->empty()) { TeEDebug(">>: 远件点云为空"); return; }

    // ---- 3. 选择近件试样 ----
    ChoseCloudDialog dNear(CloudMap, ColorMap, "焊前装配测量: 选择【近件】试样点云");
    if (dNear.exec() != QDialog::Accepted) { TeEDebug(">>: 操作取消"); return; }
    if (dNear.getSelectedList().empty()) { TeEDebug(">>: 未选择近件点云"); return; }
    const std::string nearName = dNear.getSelectedList()[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr nearCloud = CloudMap[nearName];
    if (!nearCloud || nearCloud->empty()) { TeEDebug(">>: 近件点云为空"); return; }
    if (nearName == farName) { TeEDebug(">>: 近件与远件不能是同一份点云。"); return; }

    // ---- 4. 计算(参数全部自动, 不弹参数对话框; 只考虑环缝) ----
    MeasureWeldPreparation::Params wp;
    wp.design_radius = designR;                                  // 来自所选圆柱结果
    wp.metric = MeasureWeldPreparation::Metric::Both;            // 一次配对同时给出阶差与间隙
    wp.gap_direction = MeasureWeldPreparation::GapDirection::Axial;  // 环缝: 跨缝方向固定为轴向
    auto mp = std::make_shared<MeasureWeldPreparation>();
    auto wpResult = std::make_shared<MeasureWeldPreparation::Result>();
    mp->setNearCloud(nearCloud, nearName);
    mp->setFarCloud(farCloud, farName);
    mp->setParams(wp);
    mp->setFixedReferenceAxis(axisPoint, axisDir);               // 不拟合, 直接用已保存的基准
    mp->setRoleOverride(MeasureWeldPreparation::Role::Near,
                        MeasureWeldPreparation::Role::Far, "用户在对话框中指定");

    RunAsyncVoid(QString::fromStdString(metricName),
        [this, mp, wpResult]() {
            mp->setProgressCallback(WorkerProgressCallback());
            *wpResult = mp->evaluate();
            // 调试框只留一行结论(完整报告在结束后写入报告区)
            {
                std::ostringstream oss;
                oss.setf(std::ios::fixed); oss.precision(3);
                // 只报结果, 不输出状态枚举/方法性说明
                oss << ">> 焊前装配测量: ";
                if (wpResult->maximum_step.ok || wpResult->maximum.ok) {
                    if (wpResult->maximum_step.ok)
                        oss << "径向阶差 " << wpResult->maximum_step.value << " mm";
                    if (wpResult->maximum.ok) {
                        if (wpResult->maximum_step.ok) oss << " | ";
                        oss << (wpResult->gap_direction == MeasureWeldPreparation::GapDirection::Axial
                                    ? "轴向间隙 " : "接缝法向间隙 ")
                            << wpResult->maximum.value << " mm";
                    }
                }
                else {
                    oss << "不可测";
                    if (!wpResult->reason.empty()) oss << "(" << wpResult->reason << ")";
                }
                PostLog(oss.str());
            }
        },
        [this, mp, wpResult]() {
            const auto& result = *wpResult;
            if (result.status == MeasureWeldPreparation::Status::Cancelled) {
                TeEDebug(">>: 焊前测量已取消, 不保存半成品结果。");
                return;
            }
            Update_CFmes(result.report);
            VisualizeWeldPreparation(result, mp->frame());
            TeEDebug(">>: 焊前测量完成。");
            ui->winOfAnalyzer->renderWindow()->Render();
            ui->winOfAnalyzer->update();
        });
}


// ============================================================
// 焊前装配测量 三维标注(只在 GUI 线程调用; 数据全部来自 Result + 参考圆柱框架)
//   ① 两条缝边: 醒目折线(近件绿 / 远件黄)
//   ② 点对点映射: 稀疏青色直线(有间距, 用于检查配对是否出错)
//   ③ 最大值处: 间隙=一小段弧线(红); 阶差=径向直线(蓝);
//      两边母材对应点的映射连线(橙); 各带空间文本标注
// ============================================================
void CloudForgeAnalyzer::cleanWeldPrepVisuals()
{
    // 直线段: 走全局注册(LineMap), 用工程既有 DeleteLine 移除
    for (const auto& id : m_weldPrepLineIds) DeleteLine(id);
    m_weldPrepLineIds.clear();
    // 三维文字: 工程无封装, 与凹塘一致 -> 记 id + removeText3D
    for (const auto& id : m_weldPrepShapeIds) {
        viewer->removeShape(id);
        viewer->removePointCloud(id);
        viewer->removeText3D(id);
    }
    m_weldPrepShapeIds.clear();
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    for (const auto& id : m_weldPrepActorIds) {
        auto it = ActorMap.find(id);
        if (it != ActorMap.end()) {
            if (renderer) renderer->RemoveActor(it->second);
            ActorMap.erase(it);
        }
    }
    m_weldPrepActorIds.clear();
}

void CloudForgeAnalyzer::VisualizeWeldPreparation(const MeasureWeldPreparation::Result& result,
                                                 const CylinderSurfaceFrame& frame)
{
    cleanWeldPrepVisuals();
    if (result.near_edge.refined.empty() && result.far_edge.refined.empty()) return;

    const auto proj = [&](double a, double s, double e) { return frame.unproject(a, s, e); };
    const auto radialDir = [&](double a, double s) {
        return (frame.unproject(a, s, 1.0) - frame.unproject(a, s, 0.0)).normalized();
    };

    // 字高/标记按场景尺度自适应(与其它测量一致)
    double textScale = 10.0, markScale = 3.0;
    {
        Eigen::Vector3d lo = Eigen::Vector3d::Zero(), hi = Eigen::Vector3d::Zero();
        bool first = true;
        for (const auto& e : { &result.near_edge, &result.far_edge }) {
            for (const auto& p : e->points) {
                if (!p.valid) continue;
                if (first) { lo = hi = p.xyz; first = false; }
                else { lo = lo.cwiseMin(p.xyz); hi = hi.cwiseMax(p.xyz); }
            }
        }
        if (!first) {
            const double diag = (hi - lo).norm();
            textScale = std::min(std::max(0.022 * diag, 6.0), 14.0);
            markScale = std::min(std::max(0.006 * diag, 2.0), 4.5);
        }
    }

    auto addPolyline = [&](const std::vector<Eigen::Vector3d>& pts, const std::string& id,
                           double r, double g, double b, double width) {
        if (pts.size() < 2) return;
        vtkSmartPointer<vtkPoints> vpts = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
        std::vector<vtkIdType> ids;
        ids.reserve(pts.size());
        for (const auto& p : pts) ids.push_back(vpts->InsertNextPoint(p.x(), p.y(), p.z()));
        cells->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
        vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
        poly->SetPoints(vpts);
        poly->SetLines(cells);
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(poly);
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);
        AddActors(id, actor);
        m_weldPrepActorIds.push_back(id);
    };
    // 多条线段合成一个 actor: AddLine 每次都会 Render(), 几十条会明显卡顿
    auto addSegments = [&](const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segs,
                           const std::string& id, double r, double g, double b, double width) {
        if (segs.empty()) return;
        vtkSmartPointer<vtkPoints> vpts = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
        for (const auto& s : segs) {
            const vtkIdType i0 = vpts->InsertNextPoint(s.first.x(), s.first.y(), s.first.z());
            const vtkIdType i1 = vpts->InsertNextPoint(s.second.x(), s.second.y(), s.second.z());
            cells->InsertNextCell(2);
            cells->InsertCellPoint(i0);
            cells->InsertCellPoint(i1);
        }
        vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
        poly->SetPoints(vpts);
        poly->SetLines(cells);
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(poly);
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);
        AddActors(id, actor);
        m_weldPrepActorIds.push_back(id);
    };

    // 直线段一律走工程既有封装 AddLine -> 注册进 LineMap(可被 DeleteLine/ClearAllLines 管理)。
    // 注意: AddLine 内部按 color.r/255.0 取色, 因此 ColorManager 传 0~255。
    auto addSegment = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                          const std::string& id, double r255, double g255, double b255, double width) {
        const pcl::PointXYZ a(static_cast<float>(p0.x()), static_cast<float>(p0.y()), static_cast<float>(p0.z()));
        const pcl::PointXYZ c(static_cast<float>(p1.x()), static_cast<float>(p1.y()), static_cast<float>(p1.z()));
        AddLine(id, a, c, ColorManager(r255, g255, b255), width);
        m_weldPrepLineIds.push_back(id);
    };

    // ---- ① 两条缝边(醒目折线) ----
    for (size_t i = 0; i < result.near_edge.refined.size(); ++i) {
        std::vector<Eigen::Vector3d> poly;
        for (const auto& q : result.near_edge.refined[i]) poly.push_back(proj(q.x(), q.y(), 0.0));
        addPolyline(poly, "weldprep_near_edge_" + std::to_string(i), 0.10, 1.00, 0.20, 3.0);
    }
    for (size_t i = 0; i < result.far_edge.refined.size(); ++i) {
        std::vector<Eigen::Vector3d> poly;
        for (const auto& q : result.far_edge.refined[i]) poly.push_back(proj(q.x(), q.y(), 0.0));
        addPolyline(poly, "weldprep_far_edge_" + std::to_string(i), 1.00, 0.80, 0.05, 3.0);
    }

    // ---- ② 点对点映射: 稀疏直线(约 36 条, 不密集) ----
    {
        std::vector<const MeasureWeldPreparation::Match*> v;
        for (const auto& m : result.matches) if (m.valid) v.push_back(&m);
        if (!v.empty()) {
            const int stride = std::max<int>(1, static_cast<int>(v.size()) / 36);
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segs;
            for (size_t i = 0; i < v.size(); i += stride) {
                const auto* m = v[i];
                segs.emplace_back(m->near_xyz, proj(m->far_a, m->far_s, m->far_e));
            }
            addSegments(segs, "weldprep_map", 0.10, 0.70, 0.90, 1.0);   // 单个 actor, 一次重绘
        }
    }

    // ---- ③ 最大值处标注 ----
    auto findMatch = [&](int nearIndex) -> const MeasureWeldPreparation::Match* {
        for (const auto& m : result.matches) if (m.valid && m.near_index == nearIndex) return &m;
        return nullptr;
    };
    auto annotate = [&](const MeasureWeldPreparation::MaxItem& mx, bool isGap,
                        const char* qty, double median) {
        if (!mx.ok) return;
        const auto* mm = findMatch(mx.near_index);
        if (!mm) return;
        const std::string tag = isGap ? "gap" : "step";
        if (isGap) {
            // 间隙: 参考柱面上的一小段弧线(点对映射路径)
            std::vector<Eigen::Vector3d> arc;
            const int n = 24;
            for (int i = 0; i <= n; ++i) {
                const double f = static_cast<double>(i) / n;
                arc.push_back(proj(mx.near_a + f * (mx.far_a - mx.near_a),
                                   mx.near_s + f * (mx.far_s - mx.near_s), 0.0));
            }
            addPolyline(arc, "weldprep_" + tag + "_arc", 1.00, 0.15, 0.15, 4.0);
        }
        else {
            // 阶差: 同一站位上的径向直线, 从远件材料面到近件材料面
            const Eigen::Vector3d pf = proj(mm->near_a, mm->near_s, mm->far_e);
            const Eigen::Vector3d pn = proj(mm->near_a, mm->near_s, mm->near_e);
            addSegment(pf, pn, "weldprep_" + tag + "_radial", 51.0, 115.0, 255.0, 4.0);
        }
        // 两边母材对应点的映射连线(橙色)
        const Eigen::Vector3d qMat = proj(mm->far_a, mm->far_s, mm->far_e);
        addSegment(mm->near_xyz, qMat, "weldprep_" + tag + "_link", 255.0, 140.0, 0.0, 3.0);

        // 空间文本标注(径向向外偏移 + 引线)
        std::ostringstream lab;
        lab.setf(std::ios::fixed); lab.precision(3);
        lab << qty << " max " << mx.value << " mm  中值 " << median << " mm";
        const Eigen::Vector3d base = mm->near_xyz;
        const Eigen::Vector3d off = radialDir(mm->near_a, mm->near_s) * (markScale + 2.4 * textScale);
        const Eigen::Vector3d txt = base + off;
        const std::string tid = "weldprep_" + tag + "_text";
        viewer->removeText3D(tid);
        viewer->addText3D(lab.str(), pcl::PointXYZ(static_cast<float>(txt.x()), static_cast<float>(txt.y()),
                                                   static_cast<float>(txt.z())),
                          textScale, isGap ? 1.0 : 0.35, isGap ? 0.25 : 0.65, isGap ? 0.25 : 1.0, tid);
        m_weldPrepShapeIds.push_back(tid);
        addSegment(base, txt, "weldprep_" + tag + "_leader",
                   isGap ? 255.0 : 90.0, isGap ? 64.0 : 165.0, isGap ? 64.0 : 255.0, 1.0);
    };
    if (result.maximum.ok) annotate(result.maximum, true, "间隙", result.gap_median);
    if (result.maximum_step.ok) annotate(result.maximum_step, false, "阶差", result.step_median);

    TeEDebug(">>: 焊前装配标注完成: 近件缝边(绿) / 远件缝边(黄) / 稀疏映射线(青) / "
             "间隙弧(红) / 阶差径向线(蓝) / 对应点连线(橙)。");
}

void CloudForgeAnalyzer::Tool_MeasureHeight() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialogMeasure(CloudMap, ColorMap,"选择-测量对象");
    if (dialogMeasure.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialogMeasure.getSelectedList().empty()) {
        TeEDebug("测量已取消：未选择被测量点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr measureCloud = CloudMap[dialogMeasure.getSelectedList()[0]];
    if (!measureCloud || measureCloud->empty()) {
        TeEDebug("被测量点云为空或无效");
        return;
    }

    // 选择用于拟合参考平面的点云
    ChoseCloudDialog dialogRef(CloudMap, ColorMap,"选择-参考面");
    if (dialogRef.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialogRef.getSelectedList().empty()) {
        TeEDebug("测量已取消：未选择参考平面点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud = CloudMap[dialogRef.getSelectedList()[0]];
    if (!refCloud || refCloud->empty()) {
        TeEDebug("参考点云为空或无效");
        return;
    }
    // 弹出参数设置对话框
    ParamDialogMeasureHeight paramDialog;
    if (paramDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    bool ok1 = false, ok2 = false;
    int HEIGHT_NEIGHBOR_COUNT = paramDialog.getParams()[0].toInt(&ok1);
    int HEIGHT_ITERATIONS     = paramDialog.getParams()[1].toInt(&ok2);
    if (!ok1 || !ok2 || HEIGHT_NEIGHBOR_COUNT < 1 || HEIGHT_ITERATIONS < 1) {
        TeEDebug("测高参数无效");
        return;
    }

    // 后台执行迭代测量(界面保持响应; 可在工具栏“取消计算”中止)
    auto measurerHolder = std::make_shared<std::shared_ptr<MeasureHeight>>();
    auto avgDistances = std::make_shared<std::vector<double>>();
    auto measureOk = std::make_shared<bool>(false);

    RunAsyncVoid("高度测量",
        [this, measurerHolder, avgDistances, measureOk, measureCloud, refCloud,
         HEIGHT_NEIGHBOR_COUNT, HEIGHT_ITERATIONS]() {
            PostProgress(0, 0, "高度测量: 拟合参考平面与迭代测量...");
            // 计算类在工作线程中构造并执行(构造函数不弹框)
            *measurerHolder = std::make_shared<MeasureHeight>(measureCloud, refCloud);
            *measureOk = (*measurerHolder)->measureIterative(
                *avgDistances, HEIGHT_NEIGHBOR_COUNT, HEIGHT_ITERATIONS);
            if (!*measureOk) {
                PostLog(">>: 测高失败：无法拟合参考平面或点云无效");
            }
        },
        [this, measurerHolder, avgDistances, measureOk, measureCloud, refCloud,
         HEIGHT_ITERATIONS]() {
    if (!*measureOk) {
        TeEDebug("测高失败：无法拟合参考平面或点云无效");
        return;
    }

    // 依次输出每次迭代结果
    for (size_t i = 0; i < avgDistances->size(); ++i) {
        char buf[128];
        snprintf(buf, sizeof(buf), "测高[%zu/%d]: 平均距离 = %.4f",
            i + 1, HEIGHT_ITERATIONS, (*avgDistances)[i]);
        TeEDebug(buf);
        Update_CFmes(buf);
    }

    // 可视化(在 GUI 线程执行, VTK 不能在工作线程调用)
    visualizeMeasurementResults(**measurerHolder, measureCloud, refCloud);
        });
}
void CloudForgeAnalyzer::Tool_MeasureAngleP2P() {
    ChosePlaneDialog dialog(planeResultsMap);
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>: 未选择平面拟合结果");
        return;
    }
    if(dialog.getSelectedList().size()!=2){
        TeEDebug(">>: 请选择两个平面拟合结果");
        return;
	}

	pcl::ModelCoefficients::Ptr plane1 = planeResultsMap[dialog.getSelectedList()[0]];
    pcl::ModelCoefficients::Ptr plane2 = planeResultsMap[dialog.getSelectedList()[1]];

    if (plane1->values.size() < 4 || plane2->values.size() < 4) {
        TeEDebug(">>: 平面系数无效");
        return;
    }

    // 从平面系数中提取法向量
    Eigen::Vector3f normal1, normal2;
    normal1[0] = plane1->values[0];
    normal1[1] = plane1->values[1];
    normal1[2] = plane1->values[2];

    normal2[0] = plane2->values[0];
    normal2[1] = plane2->values[1];
    normal2[2] = plane2->values[2];

    // 归一化法向量（确保是单位向量）
    normal1.normalize();
    normal2.normalize();

    // 计算法向量的点积
    float dot_product = normal1.dot(normal2);

    // 确保点积在[-1, 1]范围内，防止浮点误差
    dot_product = std::max(-1.0f, std::min(1.0f, dot_product));

    // 计算夹角（弧度）
    float angle_rad = acosf(dot_product);

    // 转换为角度
    float angle_deg = angle_rad * 180.0f / M_PI;

    // 平面夹角通常取锐角（0-90度），如果夹角大于90度，取其补角
    if (angle_deg > 90.0f) {
        angle_deg = 180.0f - angle_deg;
    }

    // 输出结果
    std::stringstream ss;
    ss << ">>: 平面夹角测量结果:" << std::endl;
    ss << "平面1: " << dialog.getSelectedList()[0] << std::endl;
    ss << "平面2: " << dialog.getSelectedList()[1] << std::endl;
    ss << "平面1法向量: (" << normal1[0] << ", " << normal1[1] << ", " << normal1[2] << ")" << std::endl;
    ss << "平面2法向量: (" << normal2[0] << ", " << normal2[1] << ", " << normal2[2] << ")" << std::endl;
    ss << "法向量夹角: " << angle_deg << " 度 (" << angle_rad << " 弧度)" << std::endl;

    TeEDebug(ss.str().c_str());

    
    QString result = QString("平面夹角: %1 度").arg(angle_deg, 0, 'f', 2);
    QMessageBox::information(nullptr, "平面夹角测量", result);
}

void CloudForgeAnalyzer::Tool_MeasureParallel() {
	ChoseLineDialog dialog(LineMap);
    if (dialog.getSelectedList().empty()||dialog.getSelectedList().size()!=2) {
        return;
    }
    Line* line1 = new Line();
    Line* line2 = new Line();
    line1 = &LineMap[dialog.getSelectedList()[0]];
    line2 = &LineMap[dialog.getSelectedList()[1]];
    MeasurePallel measurer(line1->dir_vector,line2->dir_vector);
	float parallelism = measurer.parallelism();
	TeEDebug("平行度(夹角):" + std::to_string(parallelism) + "度");
    Update_CFmes("平行度(夹角):" + std::to_string(parallelism) + "度");
}

void CloudForgeAnalyzer::Tool_Clip() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) { 
        TeEDebug(">>:操作取消");
        return;  
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>:没有点云被选中");
        return;
    }
    const std::string clippedSourceName = dialog.getSelectedList()[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud2 = CloudMap[clippedSourceName];
    *tempcloud1 = *tempcloud2;
    
	interactivePolygonCut(tempcloud1);
    bool result = showConfirmationDialog("确认裁切", "您确定要执行此操作吗？");
    if (!result) {
        TeEDebug(">>:操作取消");
        return;
    }

    // 裁切结果文件已由交互裁切工具写入, 后台读取(界面保持响应)
    auto clipedin = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto clipedout = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto clipLoaded = std::make_shared<bool>(false);

    RunAsyncVoid("裁切点云",
        [this, clipedin, clipedout, clipLoaded]() {
            PostProgress(0, 0, "裁切: 读取裁切结果点云...");
            if (pcl::io::loadPCDFile("PCDfiles/temp/cut/inside_points.pcd", *clipedin) == -1) {
                PostLog(">>:无法加载点云文件clipedin");
                return;
            }
            if (pcl::io::loadPCDFile("PCDfiles/temp/cut/outside_points.pcd", *clipedout) == -1) {
                PostLog(">>:无法加载点云文件clipedin");
                return;
            }
            *clipLoaded = true;
        },
        [this, clipedin, clipedout, clipLoaded, clippedSourceName]() {
    if (!*clipLoaded) {
        TeEDebug(">>:无法加载裁切结果点云");
        return;
    }
    DelePointCloud(clippedSourceName);
    ColorManager color1;
    ColorManager color2;
    beginUndoBatch("裁剪点云");
    AddPointCloud(GenerateRandomName(clippedSourceName + "_clippedin"), clipedin, color1);
    AddPointCloud(GenerateRandomName(clippedSourceName + "_clippedout"), clipedout, color2);
    endUndoBatch();
        });
}

void CloudForgeAnalyzer::Slot_ph_ProtruSeg_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];

    // 参数对话框在界面线程弹出(原先在 ProtrusionSegmentation 构造函数里弹框)
    ParamDialogProtrusion psDialog;
    if (psDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3;
    ProtrusionSegmentation::Params params;
    params.heightThreshold = psDialog.getParams()[0].toFloat(&ok1);
    params.searchRadius = psDialog.getParams()[1].toFloat(&ok2);
    params.minClusterSize = psDialog.getParams()[2].toInt(&ok3);
    if (!ok1 || !ok2 || !ok3) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行突起分割, 完成后回到界面线程可视化
    auto psHolder = std::make_shared<std::shared_ptr<ProtrusionSegmentation>>();
    RunAsyncVoid("凸起/平面分割",
        [this, psHolder, tempcloud, params]() {
            if (!PostProgress(0, 0, "凸起/平面分割: 局部拟合与聚类...")) {
                PostLog(">>: 凸起/平面分割已取消。");
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = tempcloud;  // 构造函数形参为非const引用, 需要左值
            *psHolder = std::make_shared<ProtrusionSegmentation>(cloud_in, params);
            (*psHolder)->compute();
            PostLog("凸起/平面分割完成, 平面点数: " + std::to_string((*psHolder)->getPlanarCloud()->size())
                + ", 突起点数: " + std::to_string((*psHolder)->getProtrusionCloud()->size()));
            PostProgress(100, 100, "凸起/平面分割完成");
        },
        [this, psHolder]() {
    if (!*psHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 凸起/平面分割已取消，不再显示本次结果。");
        return;
    }
    ProtrusionSegmentation& ps = **psHolder;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar = ps.getPlanarCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonplanar = ps.getProtrusionCloud();
    ColorManager c1(255, 0, 0);
    ColorManager c2(0, 255, 0);
    beginUndoBatch("凸起/平面分割");
    AddPointCloud("planar", planar, c1);
    AddPointCloud("nonplanar", nonplanar, c2);
    endUndoBatch();
        });
}

void CloudForgeAnalyzer::Slot_ph_CurvSeg_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
	ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
	}
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];

    // 交互拾取点与参数对话框都留在界面线程
    TeEDebug("请选择点 (左键选点，Enter确认，ESC取消)");
    PointPickerMgr mgr(ui->winOfAnalyzer->interactor(), 1);
    const auto& pts = mgr.GetPickedPoints();
    const auto& pts_pcl = mgr.GetPickedPCLPoints();
    if (pts.size() < 1) {
        TeEDebug("点选择已取消或不足两个点");
        return;
    }

    pcl::PointXYZ picked_point = pts_pcl[0];

    // 参数对话框在界面线程弹出(原先在 CurvatureSegmentation 构造函数里弹框)
    ParamDialogCurvSeg csDialog;
    if (csDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3, ok4;
    CurvatureSegmentation::CurvParams params;
    params.kSearch = csDialog.getParams()[0].toInt(&ok1);
    params.smoothThreshold = csDialog.getParams()[1].toFloat(&ok2);
    params.curvatureThreshold = csDialog.getParams()[2].toFloat(&ok3);
    params.minClusterSize = csDialog.getParams()[3].toInt(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行曲率区域生长分割, 完成后回到界面线程可视化
    auto csHolder = std::make_shared<std::shared_ptr<CurvatureSegmentation>>();
    RunAsyncVoid("曲率分割",
        [this, csHolder, tempcloud, picked_point, params]() {
            if (!PostProgress(0, 0, "曲率分割: 法线/曲率估计与区域生长...")) {
                PostLog(">>: 曲率分割已取消。");
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = tempcloud;  // 构造函数形参为非const引用, 需要左值
            *csHolder = std::make_shared<CurvatureSegmentation>(cloud_in, picked_point, params);
            (*csHolder)->compute();
            PostLog("曲率分割完成, 输出点数: " + std::to_string((*csHolder)->getOutputCloud()->size()));
            PostProgress(100, 100, "曲率分割完成");
        },
        [this, csHolder]() {
    if (!*csHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 曲率分割已取消，不再显示本次结果。");
        return;
    }
    CurvatureSegmentation& cs = **csHolder;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane = cs.getOutputCloud();
    Update_CFmes(cs.message);
    ColorManager randomColor;
	AddPointCloud(GenerateRandomName("planar"), plane, randomColor);
        });
}


void CloudForgeAnalyzer::Update_PointCounts() {
    int num = 0;
    for (const auto& cloud : CloudMap) {
		num += cloud.second->points.size();
    }
	std::string countText = "总点数:" + std::to_string(num);
    ui->label_countpoints->setText(QString::fromStdString(countText));
}


void CloudForgeAnalyzer::Tool_MeasureGeodisic() {

    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    TeEDebug("请选择点 (左键选点，Enter确认，ESC取消)");
    PointPickerMgr mgr(ui->winOfAnalyzer->interactor(), 2);
    const auto& pts = mgr.GetPickedPoints();
	const auto& pts_pcl = mgr.GetPickedPCLPoints();
    if (pts.size() < 2) {
        TeEDebug("点选择已取消或不足两个点");
        return;
    }
    double p0[3], p1[3];
	pcl::PointXYZ pointstart, pointend;
    std::copy(pts[0].begin(), pts[0].end(), p0);std::copy(pts[1].begin(), pts[1].end(), p1);
	pointstart = pts_pcl[0];
	pointend = pts_pcl[1];

    std::string msg="选择点: "
        + std::to_string(p0[0]) + "," + std::to_string(p0[1]) + "," + std::to_string(p0[2])
        + " 和 "
		+ std::to_string(p1[0]) + "," + std::to_string(p1[1]) + "," + std::to_string(p1[2]);
    TeEDebug(msg);

    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
    }
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];
    ParamDialogMeaGeodetic dialog2;
    double base_radius = 0.05;
	bool ok = false;
    if (dialog2.exec() == QDialog::Accepted) // 如果用户点击了“确定”
    {
        QString param = dialog2.getParams()[0]; // 获取输入的参数
        base_radius = param.toFloat(&ok);
        if (!ok) {
            qDebug() << "无效数字";
            return;
        }
        qDebug() << "参数：" << param;
    }
    else
    {
        qDebug() << "取消操作";
        return;
    }
    // 交互拾取与参数收集已在界面线程完成, 计算放到后台线程(界面保持响应)
    auto measurerHolder = std::make_shared<std::shared_ptr<GeodesicArcMeasurer>>();
    auto resultPtr = std::make_shared<GeodesicArcMeasurer::MeasurementResult>();

    RunAsyncVoid("测地线弧长测量",
        [this, measurerHolder, resultPtr, tempcloud, base_radius, pointstart, pointend]() {
            PostProgress(0, 0, "测地线弧长: 曲面重建与最短路径搜索...");
            // 计算类在工作线程中构造(MLS 曲面重建耗时)并执行; 构造函数不弹对话框
            *measurerHolder = std::make_shared<GeodesicArcMeasurer>(tempcloud, base_radius);  // 基准半径0.03m
            //(*measurerHolder)->setCurvatureRadius(0.02);  // 曲率计算半径
            //(*measurerHolder)->setGeodesicRadius(0.04);   // 测地线搜索半径
            *resultPtr = (*measurerHolder)->measureArcLength(pointstart, pointend);
            if (resultPtr->success) {
                PostLog("测地线弧长: " + std::to_string(resultPtr->arc_length) + " mm");
            }
            else {
                PostLog(">>: 测地线弧长计算失败");
            }
        },
        [this, measurerHolder, resultPtr]() {
    GeodesicArcMeasurer& measurer = **measurerHolder;
    const auto& result = *resultPtr;
    if (result.success) {
            std::string mesg2 = "测地线弧长: " + std::to_string(result.arc_length)+" mm";
            TeEDebug(mesg2);
            Update_CFmes(mesg2);

 //           4. 可视化
            auto [actors3D, textActors] = measurer.createVisualizationActors(
                result,
                true,  // 显示曲面
                true,  // 显示路径
                false, // 不显示原始点云
                false   // 不显示长度标注
            );

            // 将actors添加到VTK渲染器...
            vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();

            cleanGeodesicVisualization();
            // 添加 3D actors（先设置不可拾取，避免干扰后续点拾取）
            // 添加 3D actors（先设置不可拾取，避免干扰后续点拾取）
            for (auto& actor : actors3D) {
                if (actor) {
                    actor->PickableOff();
                    renderer->AddViewProp(actor);
                    // 保存Actor指针，以便后续清除
                    m_geodesicVisualizationActors.push_back(actor);
                }
            }
            // 添加 2D 文本 actors（也设置不可拾取）
            for (auto& textActor : textActors) {
                if (textActor) {
                    textActor->PickableOff();
                    renderer->AddViewProp(textActor);
                    // 保存文本Actor指针
                    m_geodesicVisualizationActors.push_back(textActor);
                }
            }

            // 刷新渲染
            ui->winOfAnalyzer->renderWindow()->Render();
            ui->winOfAnalyzer->update();
        }
       else{
           TeEDebug("计算失败");
       }
        });
}


void CloudForgeAnalyzer::Slot_fit_line_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CloudMap[dialog.getSelectedList()[0]];

    // 参数对话框在界面线程弹出(原先在 Fit_Line 构造函数里弹框)
    ParamDialog_FittingLine fitDialog;
    if (fitDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2;
    Fit_Line::FitParams fp;
    fp.DistanceThreshold = fitDialog.getParams()[0].toFloat(&ok1);
    fp.MaxIterations = fitDialog.getParams()[1].toInt(&ok2);
    if (!ok1 || !ok2) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行直线拟合(RANSAC), 完成后回到界面线程可视化
    auto fitterHolder = std::make_shared<std::shared_ptr<Fit_Line>>();
    RunAsyncVoid("直线拟合",
        [this, fitterHolder, cloud, fp]() {
            if (!PostProgress(0, 0, "直线拟合: RANSAC...")) {
                PostLog(">>: 直线拟合已取消。");
                return;
            }
            // 计算类在工作线程中构造(参数构造函数只保存参数, 点云拷贝也在此完成)并执行
            *fitterHolder = std::make_shared<Fit_Line>(cloud, fp);
            (*fitterHolder)->compute();
            const Eigen::VectorXf c = (*fitterHolder)->Get_Coeff_in();
            if (c.size() >= 6) {
                PostLog("直线上一点: (" + std::to_string(c[0]) + ", " + std::to_string(c[1]) + ", " + std::to_string(c[2]) + ")");
                PostLog("方向向量: (" + std::to_string(c[3]) + ", " + std::to_string(c[4]) + ", " + std::to_string(c[5]) + ")");
            }
            PostProgress(100, 100, "直线拟合完成");
        },
        [this, fitterHolder]() {
    if (!*fitterHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 直线拟合已取消，不再显示本次结果。");
        return;
    }
    Fit_Line& fitter = **fitterHolder;
    if (fitter.Get_Coeff_in().size() < 6) {
        TeEDebug(">>: 直线拟合失败：未能获得有效的直线模型系数");
        return;
    }

    // 获取结果并显示
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers = fitter.Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers = fitter.Get_Outliers();

    ColorManager color1(0, 255, 0); // 内点绿色
    ColorManager color2(255, 0, 0); // 外点红色

    beginUndoBatch("直线拟合");
    AddPointCloud("line_inliers", inliers, color1);
    AddPointCloud("line_outliers", outliers, color2);
    endUndoBatch();

    // 可视化直线
    Eigen::VectorXf coeffs = fitter.Get_Coeff_in();
    const pcl::PointXYZ& start = fitter.Get_StartPoint();
    const pcl::PointXYZ& end = fitter.Get_EndPoint();
    
    // 使用AddLine函数添加直线
    ColorManager lineColor(255, 0, 0); // 红色
	std::string lineName = GenerateRandomName("fitted_line_");
    AddLine(lineName, start, end, lineColor, 3.0, coeffs);

    std::string msg="直线上一点: (" + std::to_string(coeffs[0]) + ", " + std::to_string(coeffs[1]) + ", " + std::to_string(coeffs[2]) + ")\n"
        + "方向向量: (" + std::to_string(coeffs[3]) + ", " + std::to_string(coeffs[4]) + ", " + std::to_string(coeffs[5]) + ")";
    Update_CFmes(msg);
    TeEDebug(msg);
        });
}

void CloudForgeAnalyzer::Slot_fit_cy_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    // 参数对话框在界面线程弹出(原先在 Fit_Cylinder 构造函数里弹框)
    ParamDialog_FittingCylinder fitDialog;
    if (fitDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3, ok4;
    Fit_Cylinder::FitParams fp;
    fp.KSearch = fitDialog.getParams()[0].toInt(&ok1);
    fp.DistanceThreshold = fitDialog.getParams()[1].toFloat(&ok2);
    fp.MaxIterations = fitDialog.getParams()[2].toInt(&ok3);
    fp.InitialRadius = fitDialog.getParams()[3].toFloat(&ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行拟合(法线估计+RANSAC), 完成后回到界面线程可视化
    auto fcy = std::make_shared<Fit_Cylinder>(Cloud_Temp, fp);
    RunAsyncVoid("圆柱拟合",
        [this, fcy]() {
            PostProgress(0, 0, "圆柱拟合: 法线估计与RANSAC...");
            fcy->compute();
        },
        [this, fcy]() {
    if (fcy->isCancelled) {
        TeEDebug(">>:操作取消");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Inliers, Cloud_Outliers;
    Cloud_Inliers = fcy->Get_Inliers();
    Cloud_Outliers = fcy->Get_Outliers();
    if (Cloud_Inliers->empty()) {
        qDebug() << "圆柱拟合结果为空";
        return;
    }
    ColorManager color1(0, 255, 0);
    beginUndoBatch("圆柱拟合");
    AddPointCloud("incylinder",Cloud_Inliers, color1);

    ColorManager color2(255, 0, 0);
    AddPointCloud("outofcylinder", Cloud_Outliers, color2);
    endUndoBatch();

    Eigen::VectorXf coeff1;//, coeff2;

    coeff1 = fcy->Get_Coeff_in();
    pcl::ModelCoefficients::Ptr cylinder_coeff(new pcl::ModelCoefficients);
    cylinder_coeff->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cylinder_coeff->values[i] = coeff1(i);
	viewer->addCylinder(*cylinder_coeff, "fitted_cylinder");

    vtkSmartPointer<vtkLineSource> lineSource1 = vtkSmartPointer<vtkLineSource>::New();
    Eigen::Vector3f axis_point(coeff1[0], coeff1[1], coeff1[2]);
    Eigen::Vector3f axis_dir(coeff1[3], coeff1[4], coeff1[5]);
    axis_dir.normalize(); // 确保方向向量是单位向量

    float line_length = 800.0f;
    Eigen::Vector3f p1 = axis_point - axis_dir * line_length;
    Eigen::Vector3f p2 = axis_point + axis_dir * line_length;

    lineSource1->SetPoint1(p1.x(), p1.y(), p1.z());
    lineSource1->SetPoint2(p2.x(), p2.y(), p2.z());
    lineSource1->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper1->SetInputConnection(lineSource1->GetOutputPort());

    vtkSmartPointer<vtkActor> lineActor1 = vtkSmartPointer<vtkActor>::New();
    lineActor1->SetMapper(mapper1);
    lineActor1->GetProperty()->SetColor(1.0, 0.0, 0.0); // 绿色
    lineActor1->GetProperty()->SetLineWidth(3.0); // 设置线粗为3

    AddActors(GenerateRandomName("axis"), lineActor1);

    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    Update_CFmes(fcy->message);
        });
}
void CloudForgeAnalyzer::Slot_fi_open_Triggered()
{
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    QString runPath = QDir::currentPath() + "/PCDfiles";

    QString file_name = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("选择文件"),
        runPath,
        "*.pcd",
        nullptr,
        QFileDialog::DontResolveSymlinks
    );

    if (file_name.isEmpty())
        return;

#ifdef _WIN32
    // 转为本地 ANSI 编码（GBK）
    QByteArray localPath = file_name.toLocal8Bit();
    std::string path = localPath.constData();
#else
    std::string path = file_name.toStdString();
#endif

    // 后台读取点云文件(界面保持响应)
    auto loadedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto loadOk = std::make_shared<bool>(false);

    RunAsyncVoid("加载PCD文件",
        [this, loadedCloud, loadOk, path]() {
            PostProgress(0, 0, "加载点云文件...");
            if (pcl::io::loadPCDFile(path, *loadedCloud) == -1) {
                PostLog(">>无法加载点云文件");
                return;
            }
            *loadOk = true;
        },
        [this, loadedCloud, loadOk]() {
    if (!*loadOk) {
        TeEDebug(">>无法加载点云文件");
        return;
    }
    *cloud = *loadedCloud;
    ColorManager color(255, 255, 255);
    ClearAllPointCloud();
    AddPointCloud("example", cloud, color);
    UpdateCamera(0, 0, 1);
        });
}

void CloudForgeAnalyzer::Slot_fi_openSTL_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    QString runPath = QDir::currentPath() + "/PCDfiles";//获取项目的根路径
    QString file_name = QFileDialog::getOpenFileName(this, QStringLiteral("选择文件"), runPath, "*.STL", nullptr, QFileDialog::DontResolveSymlinks);
    if (file_name.isEmpty()) {
        return;
    }
#ifdef _WIN32
    QByteArray stlLocalPath = file_name.toLocal8Bit();
    std::string stlPath = stlLocalPath.constData();
#else
    std::string stlPath = file_name.toStdString();
#endif
    TeEDebug("开始处理STL文件: " + stlPath);

    // 1. 后台读取STL文件信息，计算推荐参数(界面保持响应)
    auto info = std::make_shared<StlImportInfo>();
    info->fileName = QFileInfo(file_name).fileName();
    info->fileSizeMB = QFileInfo(file_name).size() / (1024.0 * 1024.0);

    RunAsyncVoid("读取STL文件信息",
        [this, info, stlPath]() {
            PostProgress(0, 0, "读取STL文件信息...");
            try {
                vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
                reader->SetFileName(stlPath.c_str());
                reader->Update();

                vtkPolyData* polyData = reader->GetOutput();
                if (!polyData) {
                    info->error = "无法读取STL文件";
                    PostLog(">>: 无法读取STL文件");
                    return;
                }

                info->triangleCount = polyData->GetNumberOfCells();
                PostLog("STL文件三角形数量: " + std::to_string(info->triangleCount));

                // 计算包围盒
                double bounds[6];
                polyData->GetBounds(bounds);
                double dx = bounds[1] - bounds[0];
                double dy = bounds[3] - bounds[2];
                double dz = bounds[5] - bounds[4];

                info->modelDiagonal = sqrt(dx * dx + dy * dy + dz * dz);
                PostLog("模型对角线长度: " + std::to_string(info->modelDiagonal) + " 米");

                // 根据模型尺寸计算推荐leaf size
                if (info->modelDiagonal > 0) {
                    if (info->modelDiagonal < 0.1) {        // 小型模型 (<10cm)
                        info->recommendedLeafSize = 0.001f;  // 1mm
                    }
                    else if (info->modelDiagonal < 1.0) {  // 中型模型 (<1m)
                        info->recommendedLeafSize = 0.002f;  // 2mm
                    }
                    else if (info->modelDiagonal < 5.0) {  // 大型模型 (<5m)
                        info->recommendedLeafSize = 0.005f;  // 5mm
                    }
                    else {                            // 超大型模型
                        info->recommendedLeafSize = 0.01f;   // 1cm
                    }

                    // 根据三角形密度微调
                    if (info->triangleCount > 0) {
                        float triangleDensity = info->triangleCount / (info->modelDiagonal * info->modelDiagonal);
                        if (triangleDensity > 10000) {  // 高密度模型
                            info->recommendedLeafSize *= 0.8f;
                        }
                        else if (triangleDensity < 1000) {  // 低密度模型
                            info->recommendedLeafSize *= 1.2f;
                        }
                    }

                    // 限制范围
                    info->recommendedLeafSize = std::max(0.0005f, std::min(0.1f, info->recommendedLeafSize));
                }

            }
            catch (const std::exception& e) {
                PostLog("读取STL文件信息失败: " + std::string(e.what()));
            }
        },
        [this, info, stlPath]() {
            // 读取完成, 回到界面线程弹出参数对话框并启动转换
            ContinueStlImportAfterInfo(info, stlPath);
        });
}

// STL 信息读取完成后的界面部分: 弹参数对话框 + 启动后台转换
void CloudForgeAnalyzer::ContinueStlImportAfterInfo(const std::shared_ptr<StlImportInfo>& info,
                                                    const std::string& stlPath)
{
    if (!info->error.isEmpty()) {
        QMessageBox::warning(this, "错误", info->error);
        return;
    }

    // 2. 创建对话框获取参数
    bool ok = false;

    // 创建自定义对话框
    QDialog dialog(this);
    dialog.setWindowTitle("这是一个STL文件，你确定要转换为点云吗？");
    dialog.setFixedSize(400, 200);

    QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

    // 文件信息
    QString fileSize = QString("%1 MB").arg(info->fileSizeMB, 0, 'f', 2);

    QLabel* infoLabel = new QLabel(
        QString("文件: %1\n大小: %2\n三角形数量: %3\n模型对角线: %4 米")
        .arg(info->fileName)
        .arg(fileSize)
        .arg(info->triangleCount)
        .arg(info->modelDiagonal, 0, 'f', 3),
        &dialog
    );
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);

    // 表面点云复选框
    QCheckBox* surfaceCheckBox = new QCheckBox("生成表面点云（2到3层）", &dialog);
    surfaceCheckBox->setChecked(true);
    mainLayout->addWidget(surfaceCheckBox);

    // Leaf size 设置
    QHBoxLayout* leafLayout = new QHBoxLayout();
    QLabel* leafLabel = new QLabel("下采样 leaf-size:", &dialog);
    QLineEdit* leafEdit = new QLineEdit(QString::number(info->recommendedLeafSize, 'f', 4), &dialog);
    leafEdit->setMaximumWidth(100);

    leafLayout->addWidget(leafLabel);
    leafLayout->addWidget(leafEdit);
    leafLayout->addStretch();
    mainLayout->addLayout(leafLayout);

    // 按钮
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* okButton = new QPushButton("转换", &dialog);
    QPushButton* cancelButton = new QPushButton("取消", &dialog);

    buttonLayout->addStretch();
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    mainLayout->addLayout(buttonLayout);

    // 连接按钮信号
    connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug("用户取消STL转换");
        return;
    }

    // 获取参数
    float leafSize = leafEdit->text().toFloat(&ok);
    if (!ok || leafSize <= 0) {
        QMessageBox::warning(this, "错误", "leaf-size 必须大于0");
        return;
    }

    bool surfaceOnly = surfaceCheckBox->isChecked();

    TeEDebug("转换参数 - leafSize: " + std::to_string(leafSize) +
        ", surfaceOnly: " + std::to_string(surfaceOnly));

    // 3. 后台执行转换(界面保持响应; 可在工具栏“取消计算”中止)
    auto convertResult = std::make_shared<StlImportResult>();
    RunAsyncVoid("STL转换点云",
        [this, convertResult, stlPath, leafSize, surfaceOnly]() {
    // 4. 开始转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Convert(new pcl::PointCloud<pcl::PointXYZ>);

    try {
        // 4.1 读取STL文件
        PostLog("开始读取STL文件...");
        PostProgress(10, 100, "读取STL文件...");

        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(stlPath.c_str());
        reader->Update();

        vtkPolyData* polyData = reader->GetOutput();
        if (!polyData || polyData->GetNumberOfPoints() == 0) {
            throw std::runtime_error("STL文件为空");
        }

        PostLog("STL文件读取成功");
        PostLog("顶点数量: " + std::to_string(polyData->GetNumberOfPoints()));
        PostLog("三角形数量: " + std::to_string(polyData->GetNumberOfCells()));

        PostProgress(20, 100, "转换网格数据...");

        // 4.2 转换为PCL网格
        pcl::PolygonMesh mesh;
        pcl::io::vtk2mesh(polyData, mesh);

        // 4.3 获取顶点
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

        PostProgress(30, 100, "获取顶点数据...");

        if (surfaceOnly) {
            // 4.4a 表面采样
            PostLog("开始表面采样...");
            PostProgress(40, 100, "表面采样...");

            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

            int totalTriangles = mesh.polygons.size();
            int processedTriangles = 0;

            int batchSize = 1000;
            int numBatches = (totalTriangles + batchSize - 1) / batchSize;
            bool cancelFlag = false;

            sampledCloud->reserve(totalTriangles * 50);

            for (int batch = 0; batch < numBatches && !cancelFlag; ++batch) {
                // 更新进度并检查是否取消(返回 false 表示用户点击了“取消计算”)
                int batchEndPreview = std::min((batch + 1) * batchSize, totalTriangles);
                int progressPreview = std::min(80, 40 + static_cast<int>(40.0f * batchEndPreview / totalTriangles));
                if (!PostProgress(progressPreview, 100,
                        "表面采样: " + std::to_string(batchEndPreview) + "/" + std::to_string(totalTriangles) + " 三角形")) {
                    cancelFlag = true;
                    PostLog("用户取消转换");
                    break;
                }

                int batchStart = batch * batchSize;
                int batchEnd = std::min((batch + 1) * batchSize, totalTriangles);

                // 并行处理当前批次
#pragma omp parallel
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZ>);

                    // 线程私有随机数生成器（rand 线程不安全，双平台 MSVC/GCC 下都有数据竞争）
                    std::mt19937 rng(std::random_device{}() ^ static_cast<unsigned>(omp_get_thread_num()));

#pragma omp for nowait
                    for (int i = batchStart; i < batchEnd; ++i) {
                        if (mesh.polygons[i].vertices.size() == 3) {
                            pcl::PointXYZ A = vertices->points[mesh.polygons[i].vertices[0]];
                            pcl::PointXYZ B = vertices->points[mesh.polygons[i].vertices[1]];
                            pcl::PointXYZ C = vertices->points[mesh.polygons[i].vertices[2]];

                            float area = 0.5f * sqrt(
                                pow((B.y - A.y) * (C.z - A.z) - (B.z - A.z) * (C.y - A.y), 2) +
                                pow((B.z - A.z) * (C.x - A.x) - (B.x - A.x) * (C.z - A.z), 2) +
                                pow((B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x), 2)
                            );

                            float trianglePerimeter =
                                sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2) + pow(B.z - A.z, 2)) +
                                sqrt(pow(C.x - B.x, 2) + pow(C.y - B.y, 2) + pow(C.z - B.z, 2)) +
                                sqrt(pow(A.x - C.x, 2) + pow(A.y - C.y, 2) + pow(A.z - C.z, 2));

                            // 根据周长和面积综合计算采样点数
                            float averageSideLength = trianglePerimeter / 3.0f;
                            int samplesBasedOnLength = static_cast<int>(averageSideLength / (leafSize * 0.5f));
                            int samplesBasedOnArea = static_cast<int>(area / (leafSize * leafSize * 0.1f));

                            // 取两者较大值
                            int samplesPerTriangle = std::max(10, std::max(samplesBasedOnLength, samplesBasedOnArea));
                            samplesPerTriangle = std::min(samplesPerTriangle, 500);

                            for (int layer = 0; layer < 3; ++layer) {
                                for (int j = 0; j < samplesPerTriangle; ++j) {
                                    float r1 = static_cast<float>(rng()) / rng.max();
                                    float r2 = static_cast<float>(rng()) / rng.max();

                                    if (r1 + r2 > 1.0f) {
                                        r1 = 1.0f - r1;
                                        r2 = 1.0f - r2;
                                    }

                                    pcl::PointXYZ point;
                                    point.x = A.x + r1 * (B.x - A.x) + r2 * (C.x - A.x);
                                    point.y = A.y + r1 * (B.y - A.y) + r2 * (C.y - A.y);
                                    point.z = A.z + r1 * (B.z - A.z) + r2 * (C.z - A.z);

                                    if (layer > 0) {
                                        Eigen::Vector3f v1(B.x - A.x, B.y - A.y, B.z - A.z);
                                        Eigen::Vector3f v2(C.x - A.x, C.y - A.y, C.z - A.z);
                                        Eigen::Vector3f normal = v1.cross(v2);
                                        normal.normalize();

                                        float offset = (layer - 1) * leafSize * 0.1f;
                                        point.x += normal.x() * offset;
                                        point.y += normal.y() * offset;
                                        point.z += normal.z() * offset;
                                    }

                                    localCloud->push_back(point);
                                }
                            }
                        }
                    }

                    // 合并到全局点云
#pragma omp critical
                    {
                        *sampledCloud += *localCloud;
                    }
                }

                // 更新进度
                int processedTriangles = batchEnd;
                int progress = 40 + static_cast<int>(40.0f * processedTriangles / totalTriangles);
                progress = std::min(progress, 80);

                PostProgress(progress, 100,
                    "表面采样: " + std::to_string(processedTriangles) + "/" + std::to_string(totalTriangles) + " 三角形");

                PostLog("采样进度: " + std::to_string(progress) + "% ("
                    + std::to_string(processedTriangles) + "/" + std::to_string(totalTriangles) + " 三角形)");
            }

            *Cloud_Convert = *sampledCloud;
            PostLog("表面采样完成，采样点数: " + std::to_string(Cloud_Convert->size()));

        }
        else {
            // 4.4b 使用所有顶点
            *Cloud_Convert = *vertices;
            PostLog("使用所有顶点，点数: " + std::to_string(Cloud_Convert->size()));
        }

        // 4.5 下采样
        if (leafSize > 0 && Cloud_Convert->size() > 0) {
            PostLog("开始下采样...");
            PostProgress(85, 100, "下采样...");

            pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
            voxelGrid.setInputCloud(Cloud_Convert);
            voxelGrid.setLeafSize(leafSize, leafSize, leafSize);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            voxelGrid.filter(*filteredCloud);

            *Cloud_Convert = *filteredCloud;

            PostLog("下采样完成，点数: " + std::to_string(Cloud_Convert->size()));
        }

        PostProgress(100, 100, "转换完成！");

    }
    catch (const std::exception& e) {
        convertResult->error = QString("STL转换失败: %1").arg(e.what());
        PostLog("转换失败: " + std::string(e.what()));
        return;
    }

    // 转换成功: 结果交回界面线程加入场景
    convertResult->cloud = Cloud_Convert;
    convertResult->ok = true;
        },
        [this, convertResult, info, leafSize, surfaceOnly]() {
            FinishStlImport(info, convertResult, leafSize, surfaceOnly);
        });
}

// STL 转换完成后的界面部分: 提示结果 + 点云加入场景
void CloudForgeAnalyzer::FinishStlImport(const std::shared_ptr<StlImportInfo>& info,
                                         const std::shared_ptr<StlImportResult>& result,
                                         float leafSize, bool surfaceOnly)
{
    if (!result->error.isEmpty()) {
        QMessageBox::warning(this, "错误", result->error);
        return;
    }
    if (!result->ok) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Convert = result->cloud;

    // 显示结果信息
    QString resultMsg = QString("转换完成！\n"
        "原始STL文件: %1\n"
        "生成点云数量: %2\n"
        "Leaf size: %3\n"
        "表面点云: %4")
        .arg(info->fileName)
        .arg(Cloud_Convert->size())
        .arg(leafSize)
        .arg(surfaceOnly ? "是" : "否");

    QMessageBox::information(this, "转换完成", resultMsg);
    TeEDebug(resultMsg.toStdString());

    //..
    ClearAllPointCloud();
    viewer->removeAllShapes();
    clearAllActors();
    ColorManager color(255, 255, 255);
    AddPointCloud(GenerateRandomName("stl_convert_cloud"), Cloud_Convert, color);
}
void CloudForgeAnalyzer::Slot_fi_add_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    QString runPath = QDir::currentPath() + "/PCDfiles";
    QStringList file_names = QFileDialog::getOpenFileNames(
        this,
        QStringLiteral("选择点云文件"),
        runPath,
        "*.pcd",
        nullptr,
        QFileDialog::DontResolveSymlinks
    );

    int totalFiles = file_names.size();
    if (totalFiles == 0) {
        TeEDebug(">>未选择文件");
        return;
    }

    // 每个文件的读取结果(点云对象在后台线程创建, 加入场景在 GUI 线程)
    struct LoadedEntry {
        std::string displayName;                        // 不含路径的文件名(用于点云命名)
        std::string fileName;                           // 含后缀的文件名(用于提示)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    };
    auto entries = std::make_shared<std::vector<LoadedEntry>>();

    // 后台批量读取点云(界面保持响应; 可在工具栏“取消计算”中止)
    RunAsyncVoid("加载点云文件",
        [this, entries, file_names, totalFiles]() {
            int currentIndex = 0;
            for (const QString& file_name : file_names) {
                if (!PostProgress(currentIndex, totalFiles,
                        "正在加载(" + std::to_string(currentIndex + 1) + "/" + std::to_string(totalFiles)
                        + ") " + QFileInfo(file_name).fileName().toStdString())) {
                    PostLog(">>加载已取消");
                    break;
                }
                currentIndex++;

                // 创建新点云对象（避免覆盖）
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                // 尝试加载文件
                QString status;
                try {
#ifdef _WIN32
                    QByteArray localPath = file_name.toLocal8Bit();
                    std::string path = localPath.constData();
#else
                    std::string path = file_name.toStdString();
#endif
                    if (pcl::io::loadPCDFile(path, *cloud) == -1) {
                        status = QString(">>加载失败: %1").arg(QFileInfo(file_name).fileName());
                    }
                    else {
                        // 提取文件名（不含路径）
                        QString displayName = QFileInfo(file_name).completeBaseName();

                        LoadedEntry entry;
                        entry.displayName = displayName.toStdString();
                        entry.fileName = QFileInfo(file_name).fileName().toStdString();
                        entry.cloud = cloud;
                        entries->push_back(entry);

                        status = QString(">>已加载: %1").arg(displayName);
                    }
                }
                catch (...) {
                    status = QString(">>加载异常: %1").arg(QFileInfo(file_name).fileName());
                }

                // 显示状态信息
                PostLog(status.toStdString());
            }
            PostProgress(totalFiles, totalFiles, "加载完成");
        },
        [this, entries, totalFiles]() {
    // 将读取到的点云加入场景(仅在 GUI 线程)
    int loadedCount = 0;
    for (const auto& entry : *entries) {
        // 生成颜色并添加点云
        ColorManager color;
        AddPointCloud(entry.displayName, entry.cloud, color);
        loadedCount++;
    }

    // 如果有成功加载的文件，更新相机
    if (loadedCount > 0) {
        UpdateCamera(0, 0, 1);
        TeEDebug(QString(">>成功加载 %1/%2 个点云文件").arg(loadedCount).arg(totalFiles).toStdString().c_str());
    }
        });
}

void CloudForgeAnalyzer::Slot_fi_saveas_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    // SaveCloudDialog 的构造函数内部会 exec() 弹框, 必须在界面线程创建
    SaveCloudDialog dialog(CloudMap, ColorMap);
    auto SaveList = dialog.getSelectedList();
    if (SaveList.empty()) return;

    QDir saveDir(QDir::current().filePath("PCDfiles"));
    if (!saveDir.exists()) saveDir.mkpath(".");
    QString defaultName = QString("cloud_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMddHHmmss"));
    QString filePath = QFileDialog::getSaveFileName(this, "保存点云文件", saveDir.filePath(defaultName), "PCD文件 (*.pcd)");
    if (filePath.isEmpty()) return;

    // 选中的点云在界面线程取出并做存在性检查(读取复选框状态必须在 GUI 线程)
    auto saveClouds = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>();
    auto infoMsg = std::make_shared<QString>();
    auto saveOk = std::make_shared<bool>(false);
    for (const auto& key : SaveList) {
        auto it = CloudMap.find(key);
        if (it == CloudMap.end()) {
            *infoMsg = QString("点云 %1 不存在").arg(QString::fromStdString(key));
            QMessageBox::critical(this, "错误", *infoMsg);
            return;
        }
        saveClouds->push_back(it->second);
    }

    // 后台执行点云合并与写文件(界面保持响应)
    RunAsyncVoid("保存点云文件",
        [this, saveClouds, infoMsg, saveOk, filePath]() {
            PostProgress(0, 0, "保存点云文件: 合并并写入...");
            pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Merged(new pcl::PointCloud<pcl::PointXYZ>);
            Cloud_Merged->clear();
            for (const auto& key : *saveClouds) {
                *Cloud_Merged += *key;
            }
            // toLocal8Bit() → 系统本地编码(中文Windows即GBK),
            // 匹配PCL内部fopen的ANSI编码预期, 避免中文路径乱码
            std::string savePath = filePath.toLocal8Bit().toStdString();
            if (QFileInfo(filePath).suffix().compare("pcd", Qt::CaseInsensitive) != 0) {
                savePath += ".pcd";
            }
            if (pcl::io::savePCDFileBinaryCompressed(savePath, *Cloud_Merged) == -1) {
                *infoMsg = "保存失败";
                *saveOk = false;
                return;
            }
            *infoMsg = QString("成功保存 %1 个点云到：\n%2")
                .arg(saveClouds->size())
                .arg(QString::fromLocal8Bit(savePath));
            *saveOk = true;
            PostLog(infoMsg->toStdString());
        },
        [this, infoMsg, saveOk]() {
    if (*saveOk) {
        QMessageBox::information(this, "成功", *infoMsg);
    }
    else {
        QMessageBox::critical(this, "错误", *infoMsg);
    }
        });
}

void CloudForgeAnalyzer::Slot_fi_save_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr nowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *nowCloud = *cloud;
    if (nowCloud->points.empty())
    {
        TeEDebug(">>SspcdF:输入点云为空");
        return;
    }
    std::string path = "PCDfiles/nowCloud.pcd";

    // 后台写文件(界面保持响应)
    auto writeOk = std::make_shared<bool>(false);
    RunAsyncVoid("保存主体点云",
        [this, nowCloud, writeOk, path]() {
            PostProgress(0, 0, "保存点云: 写入 " + path + " ...");
            std::filesystem::path targetPath = "PCDfiles";
            if (!std::filesystem::exists(targetPath)) {
                PostLog(">>SspcdF:路径不存在，正在创建...");
                // 创建文件夹
                if (std::filesystem::create_directories(targetPath)) {
                    PostLog(">>SspcdF:文件夹创建成功！");
                }
                else {
                    PostLog(">>SspcdF:文件夹创建失败！");
                }
            }
            else {
                PostLog(">>SspcdF:路径合法");
            }

            pcl::PCDWriter writer;
            writer.write(path, *nowCloud, false);
            *writeOk = true;
        },
        [this, writeOk, path]() {
    if (*writeOk) {
        TeEDebug(">>SspcdF:生成主体点云" + path);
    }
    else {
        TeEDebug(">>SspcdF:点云写入失败");
    }
        });
}

void CloudForgeAnalyzer::Slot_ed_dork_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    RmCloudDialog dcc(CloudMap,ColorMap);
    std::vector<std::string> todelete = dcc.Get_toDelete();
    if (!todelete.empty()) {
        beginUndoBatch("批量删除点云");
        for (const auto& it : todelete) {
            DelePointCloud(it);
        }
        endUndoBatch();
    }
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}
void CloudForgeAnalyzer::Slot_ed_cleangeo_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    viewer->removeAllShapes();
    //clearAllActors();

    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

void CloudForgeAnalyzer::cleanGeodesicVisualization() {
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        return;
    }

    // 从渲染器中移除所有存储的Actor
    for (auto& actor : m_geodesicVisualizationActors) {
        if (actor) {
            renderer->RemoveViewProp(actor);
        }
    }
    // 清空容器
    m_geodesicVisualizationActors.clear();

    // 刷新渲染窗口
    if (ui && ui->winOfAnalyzer) {
        ui->winOfAnalyzer->renderWindow()->Render();
        ui->winOfAnalyzer->update();
    }
    TeEDebug("已清除测地线可视化。");
}


void CloudForgeAnalyzer::Slot_ed_cleanall_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    cleanGeodesicVisualization();
    viewer->removeAllShapes();
    ClearAllPointCloud();
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (renderer) {
        vtkPropCollection* props = renderer->GetViewProps();
        props->InitTraversal();
        vtkProp* prop;
        std::vector<vtkProp*> propsToRemove;
        while ((prop = props->GetNextProp()) != nullptr) {
            if (vtkScalarBarActor::SafeDownCast(prop)) {
                propsToRemove.push_back(prop);
            }
        }
        for (auto p : propsToRemove) {
            renderer->RemoveActor2D(static_cast<vtkActor2D*>(p));
        }
    }
    clearAllArcSplineActors();
    clearAllActors();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    TeEDebug("已清除所有可视化");
}

void CloudForgeAnalyzer::Slot_ed_cleanRGB_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    ClearAllPointCloudRGB();
}

void CloudForgeAnalyzer::Slot_ed_cleangeodetic_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    cleanGeodesicVisualization();

}

void CloudForgeAnalyzer::Slot_ed_clean2DActor_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (renderer) {
        vtkPropCollection* props = renderer->GetViewProps();
        props->InitTraversal();
        vtkProp* prop;
        std::vector<vtkProp*> propsToRemove;
        while ((prop = props->GetNextProp()) != nullptr) {
            if (vtkScalarBarActor::SafeDownCast(prop)) {
                propsToRemove.push_back(prop);
            }
        }
        for (auto p : propsToRemove) {
            renderer->RemoveActor2D(static_cast<vtkActor2D*>(p));
        }
    }
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    TeEDebug("已清除二维演示");
}

void CloudForgeAnalyzer::Slot_fl_2_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;
    const std::string selectedName = dialog.getSelectedList()[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[selectedName];

    // 参数对话框在界面线程弹出(原先在 Filter_sor 构造函数里弹框)
    ParamDialog_sor sorDialog;
    if (sorDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2;
    Filter_sor::Params params;
    params.mean_k = sorDialog.getParams()[0].toInt(&ok1);
    params.std_dev_mul_thresh = sorDialog.getParams()[1].toFloat(&ok2);
    if (!ok1 || !ok2 || params.mean_k <= 0) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行统计离群滤波, 完成后回到界面线程替换原有点云
    auto fsHolder = std::make_shared<std::shared_ptr<Filter_sor>>();
    RunAsyncVoid("统计离群滤波",
        [this, fsHolder, tempcloud, params]() {
            if (!PostProgress(0, 0, "统计离群滤波: 邻域统计与剔除...")) {
                PostLog(">>: 统计离群滤波已取消。");
                return;
            }
            *fsHolder = std::make_shared<Filter_sor>(tempcloud, params);
            (*fsHolder)->compute();
            PostLog("统计离群滤波完成, 输出点数: " + std::to_string((*fsHolder)->Get_filtered()->size()));
            PostProgress(100, 100, "统计离群滤波完成");
        },
        [this, fsHolder, tempcloud, selectedName]() {
    if (!*fsHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 统计离群滤波已取消，不再显示本次结果。");
        return;
    }
    *tempcloud = *(*fsHolder)->Get_filtered();
    if (tempcloud->empty()) {
        TeEDebug("操作无效");
		return;
    }
    ColorManager color(255, 255, 255);
    DelePointCloud(selectedName);
    AddPointCloud("example", tempcloud, color);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
        });
}
void CloudForgeAnalyzer::Slot_fl_1_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;
    const std::string selectedName = dialog.getSelectedList()[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[selectedName];

    // 参数对话框在界面线程弹出(原先在 Filter_voxel 构造函数里弹框)
    ParamDialog_vg vgDialog;
    if (vgDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1;
    Filter_voxel::Params params;
    params.leafsize = vgDialog.getParams()[0].toFloat(&ok1);
    if (!ok1 || params.leafsize <= 0.0f) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行体素滤波, 完成后回到界面线程替换原有点云
    auto fvHolder = std::make_shared<std::shared_ptr<Filter_voxel>>();
    RunAsyncVoid("体素滤波",
        [this, fvHolder, tempcloud, params]() {
            if (!PostProgress(0, 0, "体素滤波: 体素下采样...")) {
                PostLog(">>: 体素滤波已取消。");
                return;
            }
            *fvHolder = std::make_shared<Filter_voxel>(tempcloud, params);
            (*fvHolder)->compute();
            PostLog("体素滤波完成, 输出点数: " + std::to_string((*fvHolder)->Get_filtered()->size()));
            PostProgress(100, 100, "体素滤波完成");
        },
        [this, fvHolder, tempcloud, selectedName]() {
    if (!*fvHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 体素滤波已取消，不再显示本次结果。");
        return;
    }
    *tempcloud = *(*fvHolder)->Get_filtered();
    if (tempcloud->empty()) {
        TeEDebug("操作无效");
        return;
    }
    ColorManager color(255, 255, 255);
    DelePointCloud(selectedName);
    AddPointCloud("example", tempcloud, color);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
        });
}
void CloudForgeAnalyzer::Slot_ph_1_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 已有计算任务进行中，请等待完成或点击工具栏“取消计算”。");
        return;
    }
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];

    // 参数对话框在界面线程弹出(原先在 Cluster 构造函数里弹框)
    ParamDialog_ec ecDialog;
    if (ecDialog.exec() != QDialog::Accepted) {
        TeEDebug(">>: 参数设置取消");
        return;
    }
    bool ok1, ok2, ok3;
    Cluster::Params params;
    params.tolerance = ecDialog.getParams()[0].toFloat(&ok1);
    params.minSize = ecDialog.getParams()[1].toFloat(&ok2);
    params.maxSize = ecDialog.getParams()[2].toFloat(&ok3);
    if (!ok1 || !ok2 || !ok3) {
        TeEDebug(">>: 无效数字");
        return;
    }

    // 后台执行欧式聚类, 完成后回到界面线程逐簇可视化
    auto csHolder = std::make_shared<std::shared_ptr<Cluster>>();
    RunAsyncVoid("欧式聚类",
        [this, csHolder, tempcloud, params]() {
            if (!PostProgress(0, 0, "欧式聚类: 邻域搜索与聚类提取...")) {
                PostLog(">>: 欧式聚类已取消。");
                return;
            }
            *csHolder = std::make_shared<Cluster>(tempcloud, params);
            (*csHolder)->compute();
            PostLog("欧式聚类完成, 聚类数: " + std::to_string((*csHolder)->GetClusterMap().size()));
            PostProgress(100, 100, "欧式聚类完成");
        },
        [this, csHolder]() {
    if (!*csHolder) {
        return;   // 工作线程未执行(开始前已取消)
    }
    if (m_asyncState && m_asyncState->cancelRequested.load()) {
        TeEDebug(">>: 欧式聚类已取消，不再显示本次结果。");
        return;
    }
    Cluster& cs = **csHolder;
    if (cs.GetClusterMap().empty() || cs.GetColorMap().empty()) {
        return;
    }
    beginUndoBatch("聚类分割");
    ClearAllPointCloud();
    auto cluster_map = cs.GetClusterMap();
    auto color_map = cs.GetColorMap();
    for (const auto& pair : cluster_map) {
        int cluster_id = pair.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pair.second;
        ColorManager cluster_color = color_map.at(cluster_id);
        qDebug() << cluster_color.r << cluster_color.g << cluster_color.b;
        AddPointCloud("cluster_" + std::to_string(cluster_id),cloud_cluster, cluster_color);
        //viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, cluster_color, "cluster_" + std::to_string(cluster_id));//这里绑定颜色有点问题导致后面颜色读不出来
    }
    endUndoBatch();
        });
}



void CloudForgeAnalyzer::Slot_ChangeVA_x() { UpdateCamera(1, 0, 0); }
void CloudForgeAnalyzer::Slot_ChangeVA_y() { UpdateCamera(0, 1, 0); }
void CloudForgeAnalyzer::Slot_ChangeVA_z() { UpdateCamera(0, 0, 1); }
void CloudForgeAnalyzer::Slot_ChangeVA_o() { UpdateCamera(0, 0, 1); }



////////////////////////////////////////////////////////////////////////////////////////////////*槽函数end*/




void CloudForgeAnalyzer::TeEDebug(std::string debugMes) {
    QString QString_debugMes = QString::fromUtf8(debugMes);
    ui->textEdit_Show_Debug->append(QString_debugMes);
}





void CloudForgeAnalyzer::UpdateCamera(int a, int b, int c) {
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        qWarning() << "无法获取渲染器";
        return;
    }
    vtkCamera* camera = renderer->GetActiveCamera();
    if (!camera) camera = vtkCamera::New();

    if (b != 0) { // Y轴视角
        camera->SetViewUp(0, 0, 1); // 使用Z轴作为UP
    }
    else if (a != 0) { // X轴视角
        camera->SetViewUp(0, 1, 0); // 使用Y轴作为UP
    }
    else { // Z轴视角
        camera->SetViewUp(0, 1, 0); // 使用Y轴作为UP
    }
    camera->SetPosition(a, b, c);
    camera->SetFocalPoint(0, 0, 0);
    camera->ComputeViewPlaneNormal();

    ui->winOfAnalyzer->renderWindow()->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
    ui->winOfAnalyzer->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();
    ui->winOfAnalyzer->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}


void CloudForgeAnalyzer::Update_CFmes(std::string cfmes) {
    QString Qcfmes = QString::fromUtf8(cfmes);

    ui->textEdit_Show_CFmes->clear();
    ui->textEdit_Show_CFmes->setPlainText(Qcfmes);
}

void CloudForgeAnalyzer::AddPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ColorManager color) {
    if (m_undoBatchLevel == 0) {
        saveUndoState("添加点云: " + name);
    }
    CloudMap.emplace(name, cloud);
    ColorMap.emplace(name,color);
    qDebug() << color.r << color.g <<color.b;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorhandler(cloud,color.r, color.g, color.b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, colorhandler, name);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();

}

void CloudForgeAnalyzer::ClearAllPointCloudRGB() {
    for (auto& pair : RGBCloudMap) {
        viewer->removePointCloud(pair.first);
        pair.second.reset();
    }
    RGBCloudMap.clear();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}


void CloudForgeAnalyzer::ClearAllPointCloud() {
    if (m_undoBatchLevel == 0) {
        saveUndoState("清除全部点云");
    }
    for (auto& pair : CloudMap) {
        pair.second.reset(); // 智能指针置空，释放点云
    }
    CloudMap.clear();
    ColorMap.clear();
    viewer->removeAllPointClouds();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    UpdateCamera(0, 0, 1);
}

void CloudForgeAnalyzer::DelePointCloud(std::string name) {
    if (m_undoBatchLevel == 0) {
        saveUndoState("删除点云: " + name);
    }
    auto it = CloudMap.find(name);
    if (it != CloudMap.end()) {
        it->second.reset(); // 智能指针置空，释放点云
        CloudMap.erase(it);
    }
    CloudMap.erase(name);
    ColorMap.erase(name);
    viewer->removePointCloud(name);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();

}

// ========== 撤销/重做实现 ==========

void CloudForgeAnalyzer::saveUndoState(const std::string& description) {
    m_undoRedoManager.pushState(description, CloudMap, ColorMap);
}

void CloudForgeAnalyzer::beginUndoBatch(const std::string& description) {
    if (m_undoBatchLevel == 0) {
        saveUndoState(description);
    }
    ++m_undoBatchLevel;
}

void CloudForgeAnalyzer::endUndoBatch() {
    if (m_undoBatchLevel > 0) {
        --m_undoBatchLevel;
    }
}

void CloudForgeAnalyzer::rebuildCloudVisualization() {
    viewer->removeAllPointClouds();
    for (const auto& pair : CloudMap) {
        auto colorIt = ColorMap.find(pair.first);
        if (colorIt == ColorMap.end()) continue;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(
            pair.second, colorIt->second.r, colorIt->second.g, colorIt->second.b);
        viewer->addPointCloud<pcl::PointXYZ>(pair.second, colorHandler, pair.first);
    }
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    UpdateCamera(0, 0, 1);
}

void CloudForgeAnalyzer::cleanWeldMeasureVisuals()
{
    // 清理 PCL 管理的形状（球体、文字、热力图点云）
    for (const auto& id : m_weldMeasureShapeIds) {
        viewer->removeShape(id);
        viewer->removePointCloud(id);
        viewer->removeText3D(id);
        RGBCloudMap.erase(id);
    }
    m_weldMeasureShapeIds.clear();

    // 清理 vtkActor2D 颜色条（不受 PCL 管理）
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (renderer) {
        vtkPropCollection* props = renderer->GetViewProps();
        props->InitTraversal();
        std::vector<vtkProp*> barsToRemove;
        vtkProp* prop;
        while ((prop = props->GetNextProp()) != nullptr) {
            if (vtkScalarBarActor::SafeDownCast(prop)) {
                barsToRemove.push_back(prop);
            }
        }
        for (auto p : barsToRemove) {
            renderer->RemoveActor2D(static_cast<vtkActor2D*>(p));
        }
    }
}

void CloudForgeAnalyzer::Slot_ed_undo_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    if (!m_undoRedoManager.canUndo()) {
        TeEDebug("撤销: 没有更多历史记录");
        return;
    }
    std::string desc = m_undoRedoManager.undo(CloudMap, ColorMap);
    cleanWeldMeasureVisuals();
    rebuildCloudVisualization();
    TeEDebug("已撤销: " + desc);
    Update_CFmes("已撤销: " + desc);
}

void CloudForgeAnalyzer::Slot_ed_redo_Triggered() {
    if (IsTaskRunning()) {
        TeEDebug(">>: 计算进行中，为保证数据安全，暂不能执行该操作。");
        return;
    }
    if (!m_undoRedoManager.canRedo()) {
        TeEDebug("重做: 没有更多历史记录");
        return;
    }
    std::string desc = m_undoRedoManager.redo(CloudMap, ColorMap);
    cleanWeldMeasureVisuals();
    rebuildCloudVisualization();
    TeEDebug("已重做: " + desc);
    Update_CFmes("已重做: " + desc);
}

void CloudForgeAnalyzer::InitializeProgressBar() {
    // 设置进度条初始状态（0-100范围，显示百分比）
    ui->progressBar->setRange(0, 100);
    ui->progressBar->setTextVisible(true);    // 显示文字
    //ui->progressBar->setAlignment(Qt::AlignRight | Qt::AlignVCenter); // 文字居中
    ResetProgressBar();  // 初始化为100%完成状态
}

void CloudForgeAnalyzer::SetProgressBarValue(int percentage, const QString& message) {
    // 确保百分比在有效范围内
    percentage = qBound(0, percentage, 100);

    // 设置进度值
    ui->progressBar->setValue(percentage);

    // 设置显示信息
    if (!message.isEmpty()) {
        ui->progressBar->setFormat(message + "%p%");
    }
    else {
        ui->progressBar->setFormat("%p%");
    }

    // 强制刷新UI
    QApplication::processEvents();
}

void CloudForgeAnalyzer::ResetProgressBar() {
    // 重置为100%完成状态
    SetProgressBarValue(100, "");
}

// ============================================================
// 后台计算任务: 计算在工作线程执行, 界面保持响应
// 进度经原子共享状态传回, GUI 线程定时器读取后刷新进度条/调试文本框
// ============================================================
void CloudForgeAnalyzer::BeginAsyncTask(const QString& title)
{
    m_asyncState = std::make_shared<AsyncTaskState>();
    m_asyncRunning = true;

    if (!m_asyncTimer) {
        m_asyncTimer = new QTimer(this);
        connect(m_asyncTimer, &QTimer::timeout, this, &CloudForgeAnalyzer::PollAsyncProgress);
    }
    m_asyncTimer->start(100);   // 100ms 轮询一次进度

    ui->action_cancel_task->setEnabled(true);
    SetProgressBarValue(0, title);
    TeEDebug(">>: 开始后台计算: " + title.toStdString() + "（界面保持可用；可点击“取消计算”中止）");
}

void CloudForgeAnalyzer::EndAsyncTask(bool cancelled)
{
    if (m_asyncTimer) {
        m_asyncTimer->stop();
    }
    // 收尾: 落最后的日志并把进度条置满
    PollAsyncProgress();
    m_asyncRunning = false;
    ui->action_cancel_task->setEnabled(false);
    if (cancelled) {
        SetProgressBarValue(0, "已取消");
        TeEDebug(">>: 计算已取消。");
    }
    else {
        ResetProgressBar();
    }
}

void CloudForgeAnalyzer::PollAsyncProgress()
{
    if (!m_asyncState) {
        return;
    }
    const int current = m_asyncState->current.load();
    const int total = m_asyncState->total.load();

    std::string stage;
    std::vector<std::string> logs;
    {
        std::lock_guard<std::mutex> lk(m_asyncState->mtx);
        stage = m_asyncState->stage;
        logs.swap(m_asyncState->pendingLog);
    }
    for (const auto& line : logs) {
        TeEDebug(line);
    }
    if (total > 0) {
        const int pct = static_cast<int>(100.0 * current / total);
        SetProgressBarValue(pct, QString::fromStdString(stage));
    }
    else if (!stage.empty()) {
        SetProgressBarValue(0, QString::fromStdString(stage));
    }
}

void CloudForgeAnalyzer::CancelCurrentTask()
{
    if (m_asyncState) {
        m_asyncState->cancelRequested.store(true);
        TeEDebug(">>: 已请求取消当前计算（将在下一个检查点中止）...");
    }
}

bool CloudForgeAnalyzer::PostProgress(int current, int total, const std::string& stage)
{
    if (!m_asyncState) {
        return true;
    }
    m_asyncState->current.store(current);
    m_asyncState->total.store(total);
    if (!stage.empty()) {
        std::lock_guard<std::mutex> lk(m_asyncState->mtx);
        m_asyncState->stage = stage;
    }
    return !m_asyncState->cancelRequested.load();
}

void CloudForgeAnalyzer::PostLog(const std::string& line)
{
    if (!m_asyncState) {
        return;
    }
    std::lock_guard<std::mutex> lk(m_asyncState->mtx);
    m_asyncState->pendingLog.push_back(line);
}

void CloudForgeAnalyzer::RunAsyncVoid(const QString& title,
                                      const std::function<void()>& work,
                                      const std::function<void()>& onFinished)
{
    BeginAsyncTask(title);
    auto* watcher = new QFutureWatcher<void>(this);
    connect(watcher, &QFutureWatcher<void>::finished, this,
        [this, watcher, onFinished]() {
            const bool cancelled = m_asyncState && m_asyncState->cancelRequested.load();
            watcher->deleteLater();
            EndAsyncTask(cancelled);
            if (onFinished && !m_shuttingDown.load()) {
                onFinished();   // 在 GUI 线程执行后续可视化/收尾
            }
        });
    watcher->setFuture(QtConcurrent::run(work));
    m_asyncFuture = watcher->future();
}

// 后台任务使用的进度回调: 只写共享状态(线程安全), 不触碰 Qt 对象
std::function<bool(int, int, const std::string&)> CloudForgeAnalyzer::WorkerProgressCallback()
{
    return [this](int current, int total, const std::string& stage) -> bool {
        return PostProgress(current, total, stage);
    };
}

void CloudForgeAnalyzer::AddLine(const std::string& name,
    const pcl::PointXYZ& start,
    const pcl::PointXYZ& end,
    const ColorManager& color,
    double width,
    Eigen::VectorXf coeffs)
{
    // 如果已存在同名直线则先删除
    DeleteLine(name);

    // 创建直线信息
    Line newline(start,end,color,width,coeffs);
    // 添加到容器
    LineMap.emplace(name, newline);

    // 添加到可视化
    viewer->addLine<pcl::PointXYZ>(start, end,
        color.r / 255.0, color.g / 255.0, color.b / 255.0,
        name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
        width, name);

    // 更新渲染
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

void CloudForgeAnalyzer::DeleteLine(const std::string& name) {
    if (LineMap.find(name) != LineMap.end()) {
        viewer->removeShape(name);
        LineMap.erase(name);
    }
}

// 清空所有直线
void CloudForgeAnalyzer::ClearAllLines() {
    for (auto& pair : LineMap) {
        viewer->removeShape(pair.first);
    }
    LineMap.clear();
}

void CloudForgeAnalyzer::visualizeMeasurementResults(MeasureHeight& measurer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr measureCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud) {
    // 创建可视化器[6](@ref)
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("高度测量可视化"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05); // 深灰色背景

    // 获取平面系数
    pcl::ModelCoefficients::Ptr plane_coeffs = measurer.GetPlaneCoefficients();

    // 1. 添加参考点云（用蓝色显示）[6](@ref)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_color(refCloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(refCloud, ref_color, "reference_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");

    // 2. 添加测量点云（用绿色到红色的渐变色显示高度）[6](@ref)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_measure_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorPointCloudByHeight(measureCloud, plane_coeffs, colored_measure_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_measure_cloud, "measure_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "measure_cloud");


    // 3. 添加拟合的平面（半透明）[1,4](@ref)
    if (plane_coeffs->values.size() >= 4) {
        // 计算平面显示的大小基于测量点云的边界[1](@ref)
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*measureCloud, centroid);

        // 创建有限大小的平面[1](@ref)
        double plane_size = calculatePlaneSize(measureCloud);
        viewer->addPlane(*plane_coeffs, centroid[0], centroid[1], centroid[2], "fitted_plane");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "fitted_plane");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "fitted_plane");
    }

    // 4. 添加连接线显示高度（可选）[6](@ref)
    addHeightLines(viewer, measureCloud, plane_coeffs);

    // 5. 添加坐标系和文本信息[6](@ref)
    viewer->addCoordinateSystem(1.0, "coord_system");

    // 添加结果文本
    std::stringstream results_text;
    results_text << "results:\n";
    results_text << "max: " << std::fixed << std::setprecision(3) << measurer.GetMaxDistance() << " mm\n";
    results_text << "min: " << measurer.GetMinDistance() << " mm\n";
    results_text << "avr: " << measurer.GetMeanDistance() << " mm";

    viewer->addText(results_text.str(), 10, 70, 24, 1.0, 1.0, 1.0, "results_text");

    // 6. 设置相机位置以获得更好的视角[6](@ref)
    viewer->initCameraParameters();
    viewer->resetCamera();
    // 添加交互说明文本
    viewer->addText("按 'r' 重置视角, 按 'q' 退出", 10, 30, 12, 1.0, 1.0, 1.0, "help_text");

    // 7. 显示可视化窗口
    // 非阻塞: 用定时器在 GUI 线程驱动 spinOnce, 避免像原来那样在此自旋阻塞主窗口
    auto visHolder = std::make_shared<pcl::visualization::PCLVisualizer::Ptr>(viewer);
    QTimer* spinTimer = new QTimer();
    QObject::connect(spinTimer, &QTimer::timeout, [visHolder, spinTimer]() {
        if (!(*visHolder) || (*visHolder)->wasStopped()) {
            spinTimer->stop();
            spinTimer->deleteLater();
            return;
        }
        (*visHolder)->spinOnce(10);
    });
    spinTimer->start(50);
}

void CloudForgeAnalyzer::colorPointCloudByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::ModelCoefficients::Ptr plane_coeffs,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud) {
    colored_cloud->points.resize(cloud->points.size());

    double a = plane_coeffs->values[0];
    double b = plane_coeffs->values[1];
    double c = plane_coeffs->values[2];
    double d = plane_coeffs->values[3];
    double denom = std::sqrt(a * a + b * b + c * c);
    if (denom == 0.0) denom = 1.0;

    // 计算高度范围用于颜色映射
    double min_height = std::numeric_limits<double>::max();
    double max_height = std::numeric_limits<double>::lowest();

    std::vector<double> heights;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& p = cloud->points[i];
        double height = std::abs(a * p.x + b * p.y + c * p.z + d) / denom;
        heights.push_back(height);
        if (height < min_height) min_height = height;
        if (height > max_height) max_height = height;
    }

    double height_range = max_height - min_height;
    if (height_range == 0) height_range = 1.0;

    // 为每个点分配颜色（从绿色到红色）
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        colored_cloud->points[i].x = cloud->points[i].x;
        colored_cloud->points[i].y = cloud->points[i].y;
        colored_cloud->points[i].z = cloud->points[i].z;

        double normalized_height = (heights[i] - min_height) / height_range;

        // 绿色(low) -> 黄色(middle) -> 红色(high)
        if (normalized_height < 0.5) {
            colored_cloud->points[i].r = static_cast<uint8_t>(255 * (normalized_height * 2));
            colored_cloud->points[i].g = 255;
            colored_cloud->points[i].b = 0;
        }
        else {
            colored_cloud->points[i].r = 255;
            colored_cloud->points[i].g = static_cast<uint8_t>(255 * (2 - normalized_height * 2));
            colored_cloud->points[i].b = 0;
        }
    }
    colored_cloud->width = cloud->width;
    colored_cloud->height = cloud->height;
}

double CloudForgeAnalyzer::calculatePlaneSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 计算点云边界框大小[1](@ref)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    double dx = max_pt.x - min_pt.x;
    double dy = max_pt.y - min_pt.y;

    // 返回较大的边界尺寸，并增加20%的边距
    return std::max(dx, dy) * 1.2;
}

void CloudForgeAnalyzer::addHeightLines(pcl::visualization::PCLVisualizer::Ptr viewer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::ModelCoefficients::Ptr plane_coeffs) {
    // 只为一小部分点添加高度线以避免过于拥挤
    int step = std::max(1, static_cast<int>(cloud->points.size() / 50));

    double a = plane_coeffs->values[0];
    double b = plane_coeffs->values[1];
    double c = plane_coeffs->values[2];
    double d = plane_coeffs->values[3];
    double denom = a * a + b * b + c * c;
    if (denom == 0.0) return;

    for (size_t i = 0; i < cloud->points.size(); i += step) {
        const auto& p = cloud->points[i];

        // 计算点到平面的投影点[5](@ref)
        double t = -(a * p.x + b * p.y + c * p.z + d) / denom;
        pcl::PointXYZ proj_pt;
        proj_pt.x = p.x + a * t;
        proj_pt.y = p.y + b * t;
        proj_pt.z = p.z + c * t;

        std::string line_id = "height_line_" + std::to_string(i);
        viewer->addLine<pcl::PointXYZ>(p, proj_pt, 0.5, 0.5, 1.0, line_id);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, line_id);
    }
}

void CloudForgeAnalyzer::addCylinderResult(const std::string& name,pcl::ModelCoefficients::Ptr coeff){
    if (!coeff) {
        qDebug() << "错误：传入的圆柱系数指针为空";
        return;
    }

    if (coeff->values.size() != 7) {
        qDebug() << "错误：圆柱系数应为7个值，实际为" << coeff->values.size();
        return;
    }

    // 检查名称是否已存在
    if (cylinderResultsMap.find(name) != cylinderResultsMap.end()) {
        TeEDebug("圆柱拟合结果 '" + name + "' 已存在，将被覆盖");
    }

    cylinderResultsMap[name] = coeff;
    TeEDebug("已添加圆柱拟合结果: " + name);
}

void CloudForgeAnalyzer::addPlaneResult(const std::string& name, pcl::ModelCoefficients::Ptr coeff) {
    if (!coeff) {
        qDebug() << "错误：传入的平面系数指针为空";
        return;
    }

    if (coeff->values.size() != 4) {
        qDebug() << "错误：平面系数应为4个值，实际为" << coeff->values.size();
        return;
    }

    // 检查名称是否已存在
    if (planeResultsMap.find(name) != planeResultsMap.end()) {
        TeEDebug("平面拟合结果 '" + name + "' 已存在，将被覆盖");
    }

    planeResultsMap[name] = coeff;
    TeEDebug("已添加平面拟合结果: " + name);
    
}

pcl::ModelCoefficients::Ptr CloudForgeAnalyzer::getCylinderResult(const std::string& name){
    auto it = cylinderResultsMap.find(name);
    if (it != cylinderResultsMap.end()) {
        return it->second;
    }
    else {
        TeEDebug("未找到圆柱拟合结果: " + name);
        return nullptr;
    }
}

bool CloudForgeAnalyzer::removeCylinderResult(const std::string& name){
    auto it = cylinderResultsMap.find(name);
    if (it != cylinderResultsMap.end()) {
        cylinderResultsMap.erase(it);
        TeEDebug("已删除圆柱拟合结果: " + name);
        return true;
    }
    return false;
}

std::vector<std::string> CloudForgeAnalyzer::getAllCylinderNames(){
    std::vector<std::string> names;
    for (const auto& pair : cylinderResultsMap) {
        names.push_back(pair.first);
    }
    return names;
}

void CloudForgeAnalyzer::clearAllCylinderResults(){
    cylinderResultsMap.clear();
    TeEDebug("已清除所有圆柱拟合结果");
}

void CloudForgeAnalyzer::addArcSplineActor(const std::string& id, vtkSmartPointer<vtkActor> actor) {
    if (!actor) {
        qDebug() << "错误：尝试添加空的弧线Actor。ID:" << QString::fromStdString(id);
        return;
    }
    // 检查ID是否已存在，若存在则先移除旧的
    if (m_arcSplineMap.find(id) != m_arcSplineMap.end()) {
        qDebug() << "警告：弧线ID'" << QString::fromStdString(id) << "'已存在，将被替换。";
        removeArcSplineActor(id);
    }
    
    m_arcSplineMap[id] = actor;
    viewer->getRendererCollection()->GetFirstRenderer()->AddActor(actor);
    qDebug() << "已添加弧线Actor，ID:" << QString::fromStdString(id);
    ui->winOfAnalyzer->renderWindow()->Render();
}

bool CloudForgeAnalyzer::removeArcSplineActor(const std::string& id) {
    auto it = m_arcSplineMap.find(id);
    if (it != m_arcSplineMap.end()) {
        vtkSmartPointer<vtkActor> actor = it->second;
        // 从渲染器中移除
        if (viewer && viewer->getRendererCollection()) {
            viewer->getRendererCollection()->GetFirstRenderer()->RemoveActor(actor);
        }
        // 从映射中删除
        m_arcSplineMap.erase(it);

        // 刷新视图
        if (ui && ui->winOfAnalyzer) {
            ui->winOfAnalyzer->renderWindow()->Render();
        }

        qDebug() << "已移除弧线Actor，ID:" << QString::fromStdString(id);
        return true;
    }
    qDebug() << "移除失败：未找到弧线Actor，ID:" << QString::fromStdString(id);
    return false;
}

void CloudForgeAnalyzer::clearAllArcSplineActors() {
    if (m_arcSplineMap.empty()) {
        return;
    }

    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        m_arcSplineMap.clear();
        return;
    }

    // 从渲染器中移除所有弧线Actor
    for (auto& pair : m_arcSplineMap) {
        renderer->RemoveActor(pair.second);
    }

    // 清空映射
    m_arcSplineMap.clear();

    // 刷新视图
    if (ui && ui->winOfAnalyzer) {
        ui->winOfAnalyzer->renderWindow()->Render();
    }

    qDebug() << "已清除所有弧线可视化对象。";
}

std::vector<std::string> CloudForgeAnalyzer::getAllArcSplineIds() const {
    std::vector<std::string> ids;
    ids.reserve(m_arcSplineMap.size());
    for (const auto& pair : m_arcSplineMap) {
        ids.push_back(pair.first);
    }
    return ids;
}

vtkSmartPointer<vtkActor> CloudForgeAnalyzer::getArcSplineActor(const std::string& id) {
    auto it = m_arcSplineMap.find(id);
    if (it != m_arcSplineMap.end()) {
        return it->second;
    }
    return nullptr;
}

// CloudForgeAnalyzer.cpp
void CloudForgeAnalyzer::visualizeFittedPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const pcl::ModelCoefficients::Ptr& plane_coeffs,
    const std::string& plane_id,
    double r, double g, double b,
    double opacity) {
    if (!cloud || cloud->empty() || !plane_coeffs || plane_coeffs->values.size() < 4) {
        TeEDebug("visualizeFittedPlane: 输入点云或平面参数无效。");
        return;
    }

    // 1. 获取平面方程参数 Ax + By + Cz + D = 0
    float A = plane_coeffs->values[0];
    float B = plane_coeffs->values[1];
    float C = plane_coeffs->values[2];
    float D = plane_coeffs->values[3];
    Eigen::Vector3f plane_normal(A, B, C);
    float norm = plane_normal.norm();
    if (norm < 1e-6f) {
        TeEDebug("visualizeFittedPlane: 平面法向量无效。");
        return;
    }
    plane_normal /= norm; // 归一化
    A = plane_normal[0]; B = plane_normal[1]; C = plane_normal[2];
    D /= norm; // 同时归一化 D

    // 2. 将点云中所有点投影到该拟合平面上，并计算投影点的边界
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    Eigen::Vector3f plane_origin_point; // 平面上任意一点，用于构建局部坐标系

    bool first_point = true;
    for (const auto& point : cloud->points) {
        if (!pcl::isFinite(point)) continue;

        // 计算点到平面的有符号距离
        float dist = A * point.x + B * point.y + C * point.z + D;
        // 计算投影点坐标: P_proj = P - dist * N
        Eigen::Vector3f proj_point;
        proj_point[0] = point.x - dist * A;
        proj_point[1] = point.y - dist * B;
        proj_point[2] = point.z - dist * C;

        if (first_point) {
            plane_origin_point = proj_point;
            min_x = max_x = 0.0;
            min_y = max_y = 0.0;
            first_point = false;
            continue;
        }

        Eigen::Vector3f vec_in_plane = proj_point - plane_origin_point;
        Eigen::Vector3f ref_vec(1.0f, 0.0f, 0.0f);
        if (std::abs(plane_normal.dot(ref_vec)) > 0.9f) {
            ref_vec = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        }
        Eigen::Vector3f local_x = ref_vec - plane_normal * plane_normal.dot(ref_vec);
        local_x.normalize();
        Eigen::Vector3f local_y = plane_normal.cross(local_x);
        local_y.normalize();

        float proj_x = vec_in_plane.dot(local_x);
        float proj_y = vec_in_plane.dot(local_y);

        if (proj_x < min_x) min_x = proj_x;
        if (proj_x > max_x) max_x = proj_x;
        if (proj_y < min_y) min_y = proj_y;
        if (proj_y > max_y) max_y = proj_y;
    }

    if (first_point) {
        TeEDebug("visualizeFittedPlane: 没有有效的点用于计算投影边界。");
        return;
    }


    double center_local_x = (min_x + max_x) / 2.0;
    double center_local_y = (min_y + max_y) / 2.0;
    Eigen::Vector3f ref_vec(1.0f, 0.0f, 0.0f);
    if (std::abs(plane_normal.dot(ref_vec)) > 0.9f) {
        ref_vec = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }
    Eigen::Vector3f local_x = ref_vec - plane_normal * plane_normal.dot(ref_vec);
    local_x.normalize();
    Eigen::Vector3f local_y = plane_normal.cross(local_x);
    local_y.normalize();

    Eigen::Vector3f plane_center_world = plane_origin_point + local_x * center_local_x + local_y * center_local_y;

    double width = (max_x - min_x) * 1.05;
    double height = (max_y - min_y) * 1.05;
    if (width < 1e-6 || height < 1e-6) {
        width = height = 1.0; // 防止过小
    }

    viewer->removeShape(plane_id);

    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetOrigin(plane_center_world[0] - width/2*local_x[0] - height/2*local_y[0],
                           plane_center_world[1] - width/2*local_x[1] - height/2*local_y[1],
                           plane_center_world[2] - width/2*local_x[2] - height/2*local_y[2]);
    planeSource->SetPoint1(plane_center_world[0] + width/2*local_x[0] - height/2*local_y[0],
                           plane_center_world[1] + width/2*local_x[1] - height/2*local_y[1],
                           plane_center_world[2] + width/2*local_x[2] - height/2*local_y[2]);
    planeSource->SetPoint2(plane_center_world[0] - width/2*local_x[0] + height/2*local_y[0],
                           plane_center_world[1] - width/2*local_x[1] + height/2*local_y[1],
                           plane_center_world[2] - width/2*local_x[2] + height/2*local_y[2]);
    planeSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(planeSource->GetOutput());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    actor->GetProperty()->SetOpacity(opacity);

    AddActors(plane_id,actor);


    TeEDebug("可视化平面 '" + plane_id + "' 已完成。颜色(" +
        std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) +
        "), 不透明度 " + std::to_string(opacity) + "。");

    // 7. 刷新视图
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

void CloudForgeAnalyzer::clearAllActors() {
    if (ActorMap.empty()) {
        return;
    }

    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    if (!renderer) {
        ActorMap.clear();
        return;
    }

    // 从渲染器中移除所有在映射表中的平面Actor
    for (auto& pair : ActorMap) {
        renderer->RemoveActor(pair.second);
    }

    // 清空管理映射表
    ActorMap.clear();

    // 刷新视图
    if (ui && ui->winOfAnalyzer) {
        ui->winOfAnalyzer->renderWindow()->Render();
    }

    qDebug() << "已清除所有统一管理的平面可视化Actor。";
    TeEDebug("已清除所有平面可视化。");
}

void CloudForgeAnalyzer::AddActors(std::string id, vtkSmartPointer<vtkActor> actor) {
    viewer->getRendererCollection()->GetFirstRenderer()->AddActor(actor);
    auto it = ActorMap.find(id);
    if (it != ActorMap.end()) {
        viewer->getRendererCollection()->GetFirstRenderer()->RemoveActor(it->second);
        ActorMap.erase(it);
        qDebug() << "visualizeFittedPlane: 替换已存在的平面Actor，ID:" << QString::fromStdString(id);
    }
    ActorMap[id] = actor;
}

