#include "CloudForgeAnalyzer.h"

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
    AddPointCloud("example", cloud, color);
    
}

CloudForgeAnalyzer::~CloudForgeAnalyzer()
{
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
    connect(ui->action_fi_open, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_open_Triggered);
    connect(ui->action_fi_save, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_save_Triggered);
    connect(ui->action_fi_saveas, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_saveas_Triggered);
    connect(ui->action_fi_add, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_add_Triggered);
    connect(ui->action_ph_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_1_Triggered);
    connect(ui->action_CurvSeg, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_CurvSeg_Triggered);
    connect(ui->action_ProtruSeg, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_ProtruSeg_Triggered);
    connect(ui->action_fl_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_1_Triggered);
    connect(ui->action_fl_2, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_2_Triggered);
    connect(ui->action_fit_cy, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_cy_Triggered);
    connect(ui->action_fit_line, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_line_Triggered);
    connect(ui->measure_cylinder, &QAction::triggered,this, &CloudForgeAnalyzer::Tool_MeasureArc);
    connect(ui->measure_geodisic, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureGeodisic);
    connect(ui->measure_parallel, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureParallel);
    connect(ui->measure_height, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureHeight);
    connect(ui->measure_Cylindricity, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureCylindricity);
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
void CloudForgeAnalyzer::Tool_MeasureArc() {
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

   TeEDebug("请选择高度");
    PointPickerMgr mgr(ui->winOfAnalyzer->interactor(), 1);
    auto pts_pcl = mgr.GetPickedPCLPoints();

    if (pts_pcl.size() < 1) {
        TeEDebug("点选择已取消或不足一个点");
        return;
    }

    MeasureArc measurer(Cloud_Temp, cy_Temp,&pts_pcl[0]);
    if (measurer.isCancelled) {
        TeEDebug(">>: 操作取消");
        return;
    }

    if (measurer.success && (!measurer.isCancelled)) {
        // 1. 获取计算生成的曲线Actor
        vtkSmartPointer<vtkActor> splineActor = measurer.getVisualizationActor();

        if (splineActor) {
            // 2. 生成一个唯一的ID用于管理
            std::string actorId = GenerateRandomName("arc_spline_");

            addArcSplineActor(actorId, splineActor);

            TeEDebug(">>: 弧长曲线已成功添加到3D视图。");

            std::string resultMsg = "截面弧长: " + std::to_string(measurer.arcLength) + " 单位";
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
}


void CloudForgeAnalyzer::visualizeCylindricityHeatMap(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap_cloud,
    double min_distance, double max_distance)
{
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
        scalarBar->SetTitle(" ");
        scalarBar->SetNumberOfLabels(5); // 主标签数量
        scalarBar->SetMaximumNumberOfColors(256);


        const int titleFontSize = 60;        // 颜色条标题文字大小
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
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>:未选择点云");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    Fit_Cylinder fcy(Cloud_Temp);
    if (fcy.isCancelled) {
        TeEDebug(">>:操作取消");
        return;
    }

    Eigen::VectorXf coeff1;
    coeff1 = fcy.Get_Coeff_in();

    pcl::ModelCoefficients::Ptr cycoeff1(new pcl::ModelCoefficients);
    cycoeff1->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cycoeff1->values[i] = coeff1(i);
    viewer->addCylinder(*cycoeff1, "opted_cylinder1");
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    Update_CFmes(fcy.message);

    Eigen::Vector3f center = fcy.get_center_point();
    Eigen::Vector3f axis = fcy.get_axis_direction();

    ChoseCloudDialog dialog1(CloudMap, ColorMap);
    if (dialog1.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog1.getSelectedList().empty()) {
        TeEDebug("圆柱度评估已取消：未选择点云");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = CloudMap[dialog1.getSelectedList()[0]];
    if (!target_cloud || target_cloud->empty()) {
        TeEDebug("错误: 选择的点云为空或无效");
        return;
    }

    // 获取设计半径和容差
    bool ok1, ok2, ok3;

    ParamDialogMeausreCy pdialog;
    if (pdialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    double design_radius = pdialog.getParams()[0].toDouble(&ok1);
    double tolerance = pdialog.getParams()[1].toDouble(&ok1);
    int iters = pdialog.getParams()[2].toInt(&ok1);
    if (!ok1 || !ok2 || !ok3) {
        TeEDebug("圆柱度评估已取消");
        return;
    }

    // 创建圆柱度评估器
    MeasureCylindricity evaluator;
    evaluator.setInputCloud(target_cloud);
    evaluator.setDesignRadius(design_radius);
    evaluator.setTolerance(tolerance);


    evaluator.setInitialLine(center, axis);


    evaluator.setMaxIterations(iters);
    evaluator.setVerbose(true);

    // 执行评估
    auto result = evaluator.evaluateCylindricity();

    // 获取热力图点云和距离范围
    auto heatmap_cloud = evaluator.getHeatMapCloud();
    double min_distance, max_distance;
    evaluator.getDistanceRange(min_distance, max_distance);

    visualizeCylindricityHeatMap(heatmap_cloud, min_distance, max_distance);

    qDebug() << result.assessment_message;

    /*auto inliers = evaluator.get_inliers();
    auto outliers = evaluator.get_outliers();
    ColorManager c1(255, 0, 0);
    ColorManager c2(0, 255, 0);
    AddPointCloud("Cylindricity_Inliers", inliers, c1);
    AddPointCloud("Cylindricity_Outliers", outliers, c2);*/

    // 获取优化后的轴线参数
    Eigen::Vector3f optimized_center = result.getCylinderAxisPoint();
    Eigen::Vector3f optimized_axis = result.getCylinderAxisDirection();

    pcl::ModelCoefficients::Ptr cycoeff2(new pcl::ModelCoefficients);
    cycoeff2->values.resize(7);
    cycoeff2->values[0] = optimized_center.x();  // 轴上一点 X
    cycoeff2->values[1] = optimized_center.y();  // 轴上一点 Y
    cycoeff2->values[2] = optimized_center.z();  // 轴上一点 Z
    cycoeff2->values[3] = optimized_axis.x();    // 轴向 X
    cycoeff2->values[4] = optimized_axis.y();    // 轴向 Y
    cycoeff2->values[5] = optimized_axis.z();    // 轴向 Z
    cycoeff2->values[6] = static_cast<float>(design_radius);  // 半径

    addCylinderResult(GenerateRandomName("optimized_cylinder"), cycoeff2);
    viewer->addCylinder(*cycoeff2, "opted_cylinder2");

    qDebug() << "优化后轴线点: (" << optimized_center.x() << ", "
        << optimized_center.y() << ", " << optimized_center.z() << ")" ;
    qDebug() << "优化后轴线方向: (" << optimized_axis.x() << ", "
        << optimized_axis.y() << ", " << optimized_axis.z() << ")" ;
    // 显示结果
    TeEDebug(result.assessment_message);
    Update_CFmes(result.assessment_message);

    // 可视化结果
   // visualizeCylindricityResult(result, evaluator.getInliers(), evaluator.getOutliers());
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

void CloudForgeAnalyzer::Tool_MeasureHeight() {
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

    // 执行测量
    MeasureHeight measurer(measureCloud, refCloud);
    if (!measurer.measure()) {
        TeEDebug("测高失败：无法拟合参考平面或点云无效");
        return;
    }

    // 输出结果
    std::string out = "测高结果：最大距离 = " + std::to_string(measurer.GetMaxDistance())
        + "，最小距离 = " + std::to_string(measurer.GetMinDistance())
        + "，平均距离 = " + std::to_string(measurer.GetMeanDistance());
    visualizeMeasurementResults(measurer, measureCloud, refCloud);
    TeEDebug(out);
    Update_CFmes(out);
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
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) { 
        TeEDebug(">>:操作取消");
        return;  
    }
    if (dialog.getSelectedList().empty()) {
        TeEDebug(">>:没有点云被选中");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud2 = CloudMap[dialog.getSelectedList()[0]];
    *tempcloud1 = *tempcloud2;
    
	interactivePolygonCut(tempcloud1);
    bool result = showConfirmationDialog("确认裁切", "您确定要执行此操作吗？");
    if (result) {
		DelePointCloud(dialog.getSelectedList()[0]);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipedin(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipedout(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile("PCDfiles/temp/cut/inside_points.pcd", *clipedin) == -1) {
            TeEDebug(">>:无法加载点云文件clipedin");
            return;
        }
        if (pcl::io::loadPCDFile("PCDfiles/temp/cut/outside_points.pcd", *clipedout) == -1) {
            TeEDebug(">>:无法加载点云文件clipedin");
            return;
        }
        ColorManager color1;
        ColorManager color2;
		AddPointCloud(GenerateRandomName(dialog.getSelectedList()[0] + "_clippedin"), clipedin, color1);
        AddPointCloud(GenerateRandomName(dialog.getSelectedList()[0] + "_clippedout"), clipedout, color2);
    }
    else {
        TeEDebug(">>:操作取消");
    }
    
}

void CloudForgeAnalyzer::Slot_ph_ProtruSeg_Triggered() {
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];
    ProtrusionSegmentation ps(tempcloud);
    ps.segment();
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar = ps.getPlanarCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonplanar = ps.getProtrusionCloud();
    ColorManager c1(255, 0, 0);
    ColorManager c2(0, 255, 0);
    AddPointCloud("planar", planar, c1);
    AddPointCloud("nonplanar", nonplanar, c2);
}

void CloudForgeAnalyzer::Slot_ph_CurvSeg_Triggered() {
	ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
	}
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];
    CurvatureSegmentation cs(tempcloud);
    cs.segment();
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar = cs.getPlanarCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr nonplanar = cs.getNonPlanarCloud();
	ColorManager c1(255,0 , 0);
    ColorManager c2(0, 255, 0);
    AddPointCloud("planar", planar, c1);
	AddPointCloud("nonplanar", nonplanar, c2);
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
    GeodesicArcMeasurer measurer(tempcloud, base_radius);  // 基准半径0.03m
    //measurer.setCurvatureRadius(0.02);  // 曲率计算半径
    //measurer.setGeodesicRadius(0.04);   // 测地线搜索半径
    auto result = measurer.measureArcLength(pointstart, pointend);
    if (result.success) {
            std::string mesg2 = "测地线弧长: " + std::to_string(result.arc_length);
            TeEDebug(mesg2);

 //           4. 可视化
            auto [actors3D, textActors] = measurer.createVisualizationActors(
                result,
                true,  // 显示曲面
                true,  // 显示路径
                false, // 不显示原始点云
                true   // 显示长度标注
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
}


void CloudForgeAnalyzer::Slot_fit_line_Triggered() {
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CloudMap[dialog.getSelectedList()[0]];
    Fit_Line fitter(cloud);

    // 获取结果并显示
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers = fitter.Get_Inliers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers = fitter.Get_Outliers();

    ColorManager color1(0, 255, 0); // 内点绿色
    ColorManager color2(255, 0, 0); // 外点红色

    AddPointCloud("line_inliers", inliers, color1);
    AddPointCloud("line_outliers", outliers, color2);

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
}

void CloudForgeAnalyzer::Slot_fit_cy_Triggered() {
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];

    Fit_Cylinder fcy(Cloud_Temp);
    if (fcy.isCancelled) {
        TeEDebug(">>:操作取消");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Inliers, Cloud_Outliers;
    Cloud_Inliers = fcy.Get_Inliers();
    Cloud_Outliers = fcy.Get_Outliers();
    if (Cloud_Inliers->empty()) {
        qDebug() << "圆柱拟合结果为空";
        return;
    }
    ColorManager color1(255, 0, 0);
    AddPointCloud("incylinder",Cloud_Inliers, color1);

    ColorManager color2(0, 0, 255);
    AddPointCloud("outofcylinder", Cloud_Outliers, color2);

    Eigen::VectorXf coeff1;//, coeff2;

    coeff1 = fcy.Get_Coeff_in();
    pcl::ModelCoefficients::Ptr cylinder_coeff(new pcl::ModelCoefficients);
    cylinder_coeff->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cylinder_coeff->values[i] = coeff1(i);
	viewer->addCylinder(*cylinder_coeff, "fitted_cylinder");
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    Update_CFmes(fcy.message);
}
void CloudForgeAnalyzer::Slot_fi_open_Triggered() {
    QString runPath = QDir::currentPath()+"/PCDfiles";//获取项目的根路径
    QString file_name = QFileDialog::getOpenFileName(this, QStringLiteral("选择文件"), runPath, "*.pcd", nullptr, QFileDialog::DontResolveSymlinks);
    if (pcl::io::loadPCDFile(file_name.toStdString(), *cloud) == -1) {
        TeEDebug(">>无法加载点云文件");
        return;
    }
    ColorManager color(255, 255, 255);
    ClearAllPointCloud();
    AddPointCloud("example",cloud,color);
    UpdateCamera(0, 0, 1);
}

void CloudForgeAnalyzer::Slot_fi_add_Triggered() {
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

    // 开始任务 - 显示进度
    SetProgressBarValue(0, "加载点云");

    // 用于跟踪成功加载的文件数量
    int loadedCount = 0;
    int currentIndex = 0;

    for (const QString& file_name : file_names) {
        currentIndex++;

        // 更新进度信息（包括文件名）
        QString progressText = QString("正在加载(%1/%2)\n%3")
            .arg(currentIndex)
            .arg(totalFiles)
            .arg(QFileInfo(file_name).fileName());
        SetProgressBarValue((currentIndex * 100) / totalFiles, progressText);

        // 创建新点云对象（避免覆盖）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 尝试加载文件
        QString status;
        bool success = false;
        try {
            if (pcl::io::loadPCDFile(file_name.toStdString(), *cloud) == -1) {
                status = QString(">>加载失败: %1").arg(QFileInfo(file_name).fileName());
            }
            else {
                // 提取文件名（不含路径）
                QString displayName = QFileInfo(file_name).completeBaseName();

                // 生成颜色并添加点云
                ColorManager color;
                AddPointCloud(displayName.toStdString(), cloud, color);

                status = QString(">>已加载: %1").arg(displayName);
                success = true;
                loadedCount++;
            }
        }
        catch (...) {
            status = QString(">>加载异常: %1").arg(QFileInfo(file_name).fileName());
        }

        // 显示状态信息
        TeEDebug(status.toStdString().c_str());
    }

    // 完成后重置进度条
    ResetProgressBar();

    // 如果有成功加载的文件，更新相机
    if (loadedCount > 0) {
        UpdateCamera(0, 0, 1);
        TeEDebug(QString(">>成功加载 %1/%2 个点云文件").arg(loadedCount).arg(totalFiles).toStdString().c_str());
    }
}

void CloudForgeAnalyzer::Slot_fi_saveas_Triggered() {
    SaveCloudDialog dialog(CloudMap, ColorMap);
    auto SaveList = dialog.getSelectedList();
    if (SaveList.empty()) return;

    QDir saveDir(QDir::current().filePath("PCDfiles"));
    if (!saveDir.exists()) saveDir.mkpath(".");
    QString defaultName = QString("cloud_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMddHHmmss"));
    QString filePath = QFileDialog::getSaveFileName(this, "保存点云文件", saveDir.filePath(defaultName), "PCD文件 (*.pcd)");

    QString infoMsg;
    if (!filePath.isEmpty()) {
        bool ok = dialog.SaveSelectedClouds(filePath, infoMsg);
        if (ok) {
            QMessageBox::information(this, "成功", infoMsg);
        }
        else {
            QMessageBox::critical(this, "错误", infoMsg);
        }
    }
}

void CloudForgeAnalyzer::Slot_fi_save_Triggered() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr nowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *nowCloud = *cloud;
    if (nowCloud->points.empty())
    {
        TeEDebug(">>SspcdF:输入点云为空");
        return;
    }
    pcl::PCDWriter writer;
    std::string path = "PCDfiles/nowCloud.pcd";
    std::filesystem::path targetPath = "PCDfiles";
    if (!std::filesystem::exists(targetPath)) {
        TeEDebug(">>SspcdF:路径不存在，正在创建...");
        // 创建文件夹
        if (std::filesystem::create_directories(targetPath)) {
            TeEDebug(">>SspcdF:文件夹创建成功！");
        }
        else {
            TeEDebug(">>SspcdF:文件夹创建失败！");
        }
    }
    else {
        TeEDebug(">>SspcdF:路径合法");
    }

    writer.write(path, *nowCloud, false);
    TeEDebug(">>SspcdF:生成主体点云" + path);

}

void CloudForgeAnalyzer::Slot_ed_dork_Triggered() {
    RmCloudDialog dcc(CloudMap,ColorMap);
    std::vector<std::string> todelete = dcc.Get_toDelete();
    for (const auto& it : todelete) {
        DelePointCloud(it);
    }
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}
void CloudForgeAnalyzer::Slot_ed_cleangeo_Triggered() {
    viewer->removeAllShapes();
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
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    TeEDebug("已清除所有可视化");
}

void CloudForgeAnalyzer::Slot_ed_cleanRGB_Triggered() {
    ClearAllPointCloudRGB();
}

void CloudForgeAnalyzer::Slot_ed_cleangeodetic_Triggered() {
    cleanGeodesicVisualization();

}

void CloudForgeAnalyzer::Slot_ed_clean2DActor_Triggered() {
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
    Filter_sor fs(cloud);
    *cloud = *fs.Get_filtered();
    ColorManager color(255, 255, 255);
    ClearAllPointCloud();
    AddPointCloud("example", cloud, color);
    UpdateCamera(0, 0, 1);
}
void CloudForgeAnalyzer::Slot_fl_1_Triggered() {
    Filter_voxel fv(cloud);
    *cloud = *fv.Get_filtered();
    ColorManager color(255, 255, 255);
    ClearAllPointCloud();
    AddPointCloud("example", cloud, color);
    UpdateCamera(0, 0, 1);
}
void CloudForgeAnalyzer::Slot_ph_1_Triggered() {
    ChoseCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.exec() != QDialog::Accepted) {
        TeEDebug(">>:操作取消");
        return;
    }
    if (dialog.getSelectedList().empty()) return;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];
    Cluster cs(tempcloud);
    if (cs.GetClusterMap().empty() || cs.GetColorMap().empty()) {
        return;
    }
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
    CloudMap.emplace(name, cloud);
    ColorMap.emplace(name,color);
    qDebug() << color.r << color.g <<color.b;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorhandler(cloud,color.r, color.g, color.b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, colorhandler, name);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    UpdateCamera(0, 0, 1);
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
    UpdateCamera(0, 0, 1);
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
    TeEDebug("可视化窗口已打开，按 'q' 退出查看");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
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

void CloudForgeAnalyzer::addCylinderResult(const std::string& name,
    pcl::ModelCoefficients::Ptr coeff){
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