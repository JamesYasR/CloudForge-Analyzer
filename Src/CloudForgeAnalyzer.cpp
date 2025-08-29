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
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    viewer->setupInteractor(ui->winOfAnalyzer->interactor(), ui->winOfAnalyzer->renderWindow());
    ui->winOfAnalyzer->setRenderWindow(viewer->getRenderWindow());
    QApplication::processEvents();
    ui->winOfAnalyzer->makeCurrent();
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
    connect(ui->measure_cylinder, &QAction::triggered,this, &CloudForgeAnalyzer::Tool_SetMeasureCylinder);
    connect(ui->measure_geodisic, &QAction::triggered, this, &CloudForgeAnalyzer::Tool_MeasureGeodisic);

}

void CloudForgeAnalyzer::mainLoop_Init() {
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(Update_PointCounts())); // slotCountMessage是我们需要执行的响应函数 
    timer->start(200); // 每隔1s 
}

////////////////////////////////////////////////////////////////////////////////////////////////*槽函数start*/
void CloudForgeAnalyzer::Slot_ph_ProtruSeg_Triggered() {
    ChoseCloudDialog dialog(CloudMap, ColorMap);
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

void CloudForgeAnalyzer::Tool_SetMeasureCylinder() {
    //isCylinderMeasure = checked;
    FitCloudDialog dialog(CloudMap, ColorMap);
    if (dialog.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[dialog.getSelectedList()[0]];
	MeasureArc ma;
    ma.FitCylinder(Cloud_Temp);
    float arclength = ma.calculateArcLength();
    TeEDebug("弧长为:"+std::to_string(arclength));
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
    if (dialog.getSelectedList().empty()) {
        return;
    }
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud = CloudMap[dialog.getSelectedList()[0]];
    ParamDialogMeaArc* dialog2=new ParamDialogMeaArc();
    double base_radius = 0.05;
	bool ok = false;
    if (dialog2->exec() == QDialog::Accepted) // 如果用户点击了“确定”
    {
        QString param = dialog2->getParams()[0]; // 获取输入的参数
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

            // 添加 3D actors
            for (auto& actor : actors3D) {
                renderer->AddViewProp(actor);
            }

            // 添加 2D 文本 actors
            for (auto& textActor : textActors) {
                renderer->AddViewProp(textActor);
            }

            // 刷新渲染
            ui->winOfAnalyzer->renderWindow()->Render();
            ui->winOfAnalyzer->update();
        }
       else{
           TeEDebug("计算失败");
       }
}




void CloudForgeAnalyzer::Slot_fit_cy_Triggered() {
    FitCloudDialog window(CloudMap, ColorMap);
    if (window.getSelectedList().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[window.getSelectedList()[0]];

    Fit_Cylinder fcy(Cloud_Temp);
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

    //Fit_Cylinder fcy2(Cloud_Outliers);
    Eigen::VectorXf coeff1;//, coeff2;

    coeff1 = fcy.Get_Coeff_in();
    //coeff2 = fcy2.Get_Coeff_in();
    pcl::ModelCoefficients::Ptr cylinder_coeff(new pcl::ModelCoefficients);
    cylinder_coeff->values.resize(7);
    for (std::size_t i = 0; i < 7; ++i)
        cylinder_coeff->values[i] = coeff1(i);
	viewer->addCylinder(*cylinder_coeff, "fitted_cylinder");

    Update_CFmes("圆柱1轴上一点坐标为：" + std::to_string(coeff1[0]) + "," + std::to_string(coeff1[1]) + "," + std::to_string(coeff1[2])
        + "\n圆柱轴方向为：" + std::to_string(coeff1[3]) + "," + std::to_string(coeff1[4]) + "," + std::to_string(coeff1[5])
        + "\n圆柱半径为：" + std::to_string(coeff1[6]));
        //+ "\n圆柱轴上一点坐标为：" + std::to_string(coeff2[0]) + "," + std::to_string(coeff2[1]) + "," + std::to_string(coeff2[2])
        //+ "\n圆柱轴方向的x为：" + std::to_string(coeff2[3]) + "," + std::to_string(coeff2[4]) + "," + std::to_string(coeff2[5])
        //+ "\n圆柱半径为：" + std::to_string(coeff2[6])
        //+ "\n半径差值："+ std::to_string(coeff1[6]- coeff2[6])
        //+ "\n焊接区宽度：" + std::to_string(fcy2.ComputeCylinderHeight()));
   //这部分比较临时，可以考虑去改一下

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

    auto generateColor = []() -> ColorManager {
        return ColorManager(
            rand() % 256,
            rand() % 256,
            rand() % 256
        );
        };

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
                ColorManager color = generateColor();
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
void CloudForgeAnalyzer::Slot_ed_cleanall_Triggered() {
    viewer->removeAllShapes();
    ClearAllPointCloud();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
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
    Cluster cs(cloud);
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
        ui->progressBar->setFormat(message + " %p%");
    }
    else {
        ui->progressBar->setFormat("%p%");
    }

    // 强制刷新UI
    QApplication::processEvents();
}

void CloudForgeAnalyzer::ResetProgressBar() {
    // 重置为100%完成状态
    SetProgressBarValue(100, "就绪");
}