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
    QStringList items;
    items << "圆柱" << "平面-凹陷-直线" << "Option 3";
    ui->comboBox_ChoiceCFmethod->addItems(items);
}

void CloudForgeAnalyzer::InitalizeConnects() {
    connect(ui->action_ed_dork, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ed_dork_Triggered);
    connect(ui->action_fi_open, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_open_Triggered);
    connect(ui->action_fi_save, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_save_Triggered);
    connect(ui->action_fi_saveas, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fi_saveas_Triggered);
    connect(ui->action_ph_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_ph_1_Triggered);
    connect(ui->action_fl_1, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_1_Triggered);
    connect(ui->action_fl_2, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fl_2_Triggered);
    connect(ui->action_fit_cy, &QAction::triggered, this, &CloudForgeAnalyzer::Slot_fit_cy_Triggered);
}

////////////////////////////////////////////////////////////////////////////////////////////////*槽函数start*/
void CloudForgeAnalyzer::Slot_fit_cy_Triggered() {
    Dialog_SelSingleCloud PDFCY(CloudMap, ColorMap);
    if (PDFCY.Get_Selected().empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Temp = CloudMap[PDFCY.Get_Selected()];

    Fit_Cylinder fcy1(Cloud_Temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Inliers, Cloud_Outliers;
    Cloud_Inliers = fcy1.Get_Inliers();
    Cloud_Outliers = fcy1.Get_Outliers();
    if (Cloud_Inliers->empty()) {
        qDebug() << "圆柱拟合结果为空";
        return;
    }
    ColorManager color1(255, 0, 0);
    AddPointCloud("incylinder",Cloud_Inliers, color1);

    ColorManager color2(0, 0, 255);
    AddPointCloud("outofcylinder", Cloud_Outliers, color2);

    Fit_Cylinder fcy2(Cloud_Outliers);
    Eigen::VectorXf coeff1, coeff2;

    coeff1 = fcy1.Get_Coeff_in();
    coeff2 = fcy2.Get_Coeff_in();

    Update_CFmes("圆柱1轴上一点坐标为：" + std::to_string(coeff1[0]) + "," + std::to_string(coeff1[1]) + "," + std::to_string(coeff1[2])
        + "\n圆柱轴方向为：" + std::to_string(coeff1[3]) + "," + std::to_string(coeff1[4]) + "," + std::to_string(coeff1[5])
        + "\n圆柱半径为：" + std::to_string(coeff1[6])
        + "\n圆柱轴上一点坐标为：" + std::to_string(coeff2[0]) + "," + std::to_string(coeff2[1]) + "," + std::to_string(coeff2[2])
        + "\n圆柱轴方向的x为：" + std::to_string(coeff2[3]) + "," + std::to_string(coeff2[4]) + "," + std::to_string(coeff2[5])
        + "\n圆柱半径为：" + std::to_string(coeff2[6])
        + "\n半径差值："+ std::to_string(coeff1[6]- coeff2[6])
        + "\n焊接区宽度：" + std::to_string(fcy2.ComputeCylinderHeight()));



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
void CloudForgeAnalyzer::Slot_ed_dork_Triggered() {
    Dialog_cc dcc(CloudMap,ColorMap);
    std::vector<std::string> todelete = dcc.Get_toDelete();
    for (const auto& it : todelete) {
        DelePointCloud(it);
    }
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}
void CloudForgeAnalyzer::Slot_fi_saveas_Triggered() {
    Dialog_saveas dsvas(viewer);
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


void CloudForgeAnalyzer::Slot_CurveFittingPC() {
    QString cfmod = ui->comboBox_ChoiceCFmethod->currentText().trimmed();
    if (cfmod == "圆柱") {
        //输入部分start
        TeEDebug(">>槽CFPC:开始拟合圆柱");
        pcl::PointCloud<pcl::PointXYZ>::Ptr acloud(new pcl::PointCloud<pcl::PointXYZ>);
        *acloud = *cloud;
        TeEDebug(">>槽CFPC:输入点云点数:" + std::to_string(acloud->points.size()));
        //输入部分end

        //子弹缺陷圆柱用法
        Protrusion_Depression_Cylinder PDC1(acloud, [this](const std::string& message) {
            this->TeEDebug(message); });
        PDC1.VoxelGrid_Sor_filter();
        PDC1.FitCylinderModel();
        PDC1.AnalyzeDefects(); // 新增分析
        pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
        //*newcloud = *PDC1.main_cloud;
        *newcloud = *cloud;
        //ReplacePointCloud(newcloud);
        viewer->addPointCloud(PDC1.defects_cloud, "cloud_defect");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_defect", 0);
        viewer->addCylinder(PDC1.coefficients_cylinder, "cycler", 0); // 可视化拟合出来的圆柱模型
        PDC1.MarkDefects(viewer);//标记缺陷，包括返回缺陷数

        Update_CFmes(PDC1.mainbody_fit_data + PDC1.defect_fit_data);
        ui->winOfAnalyzer->update();
    }
    else if (cfmod == "平面-凹陷-直线") {
        TeEDebug(">>槽CFPC:开始拟合平面-凹陷-直线");
        pcl::PointCloud<pcl::PointXYZ>::Ptr acloud(new pcl::PointCloud<pcl::PointXYZ>);
        *acloud = *cloud;
        TeEDebug(">>槽CFPC:输入点云点数:" + std::to_string(acloud->points.size()));
        Linear_Depression_Plane LDP1(acloud, [this](const std::string& message) {
            this->TeEDebug(message); });
        LDP1.VoxelGrid_Sor_filter();//滤波
        LDP1.FitPlaneModel();
        float scale[2] = { 200,500 };
        auto plane = createPlane(LDP1.coefficients_plane, 3, 3, 3, scale);
        viewer->addModelFromPolyData(plane, "plane_1");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "plane_1", 0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
        viewer->addPlane(LDP1.coefficients_plane, "plane");
        ui->winOfAnalyzer->update();

        Update_CFmes(LDP1.get_mainbody_fit_data() + "\n识别数量:2\n" + "平均深度1:2.3754\n" + "平均深度1:2.0353\n");//这部分还没做完哈

    }
    else {
        TeEDebug(">>槽CFPC:未定义的曲线拟合方法");
    }
}


void CloudForgeAnalyzer::Slot_ChangeVA_x() { UpdateCamera(1, 0, 0); }
void CloudForgeAnalyzer::Slot_ChangeVA_y() { UpdateCamera(0, 1, 0); }
void CloudForgeAnalyzer::Slot_ChangeVA_z() { UpdateCamera(0, 0, 1); }
void CloudForgeAnalyzer::Slot_ChangeVA_o() { UpdateCamera(0, 0, 1); }

void CloudForgeAnalyzer::Slot_CloudPointIncise() {

}


void CloudForgeAnalyzer::Slot_DoCPIC() {

}

void CloudForgeAnalyzer::Slot_CtrlZPCIC() {

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
    CloudMap.clear();
    ColorMap.clear();
    viewer->removeAllPointClouds();
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    UpdateCamera(0, 0, 1);
}

void CloudForgeAnalyzer::DelePointCloud(std::string name) {
    CloudMap.erase(name);
    ColorMap.erase(name);
    viewer->removePointCloud(name);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
    UpdateCamera(0, 0, 1);
}
