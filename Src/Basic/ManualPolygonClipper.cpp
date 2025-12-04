#include "Basic/ManualPolygonClipper.h"
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <QDebug>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

ManualPolygonClipper::ManualPolygonClipper(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    vtkSmartPointer<vtkRenderWindow> mainRenderWindow,
    vtkRenderWindowInteractor* mainInteractor, // 改为原生指针
    pcl::visualization::PCLVisualizer* mainviewer,
    QObject* parent
) : QObject(parent)
, cloud_in(input_cloud)
, cloud_polygon(new pcl::PointCloud<pcl::PointXYZ>)
, cloud_cliped(new pcl::PointCloud<pcl::PointXYZ>)
, cloud_remain(new pcl::PointCloud<pcl::PointXYZ>)
, flag(false)
, isPickingMode(false)
, line_id(0)
, viewer(mainviewer)
{
    // 保存传入的交互器指针（不接管其生命周期）
    this->mainInteractor = mainInteractor;

    if (!mainInteractor) {
        qWarning() << "ManualPolygonClipper: 构造时 mainInteractor 为 nullptr，功能可能不可用";
    }
    else {
        // 确保 interactor 已初始化并启用（但不要调用 Start()）
        if (!mainInteractor->GetEnabled()) mainInteractor->Enable();
        mainInteractor->Initialize();

        // 仅将 PCLVisualizer 绑定到现有 interactor/renderWindow，避免 SetInteractorStyle 改变全局样式
        viewer->setupInteractor(mainInteractor, mainRenderWindow);
    }

    // 添加用于绘制的点云和被裁剪点云
    viewer->addPointCloud(cloud_polygon, "polyline");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polyline");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "polyline");
    viewer->addPointCloud(cloud_in, "cloud_in");

    // 使用 PCLVisualizer 的回调接口，保存返回的连接以便之后断开
    keyboard_connection = viewer->registerKeyboardCallback(&ManualPolygonClipper::keyboardCallback, this);
    mouse_connection = viewer->registerMouseCallback(&ManualPolygonClipper::mouseCallback, this);
}

ManualPolygonClipper::~ManualPolygonClipper() {
    cleanup();
}

void ManualPolygonClipper::cleanup() {
    // 注：不要在 cleanup 中替换或释放 QVTK 提供的 interactor，
    //      仅断开回调连接，移除临时可视化对象，保持 interactor 生命周期由 QVTK 管理。

    if (viewer) {
        // 正确断开回调连接（而不是传入 nullptr）
        if (keyboard_connection.connected()) keyboard_connection.disconnect();
        if (mouse_connection.connected()) mouse_connection.disconnect();

        // 移除临时点云/形状
        viewer->removePointCloud("polyline");
        viewer->removePointCloud("aftercut");

        // 移除所有线段（line_id 代表已添加的线段计数）
        for (int i = 0; i < static_cast<int>(line_id); ++i) {
            char str[512];
            sprintf(str, "line#%03d", i);
            viewer->removeShape(str);
        }
    }

    // 不要调用 mainInteractor->SetInteractorStyle(...) 或 SetRenderWindow(nullptr)
    // 保持 mainInteractor 由 QVTK 管理，避免悬空或被销毁
}

void ManualPolygonClipper::startClipping() {
    resetPolygon();
    isPickingMode = true;
    viewer->getRenderWindow()->Render();  // 触发渲染
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ManualPolygonClipper::getClippedCloud() const {
    return cloud_cliped;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ManualPolygonClipper::getRemainCloud() const {
    return cloud_remain;
}

void ManualPolygonClipper::resetPolygon() {
    line_id = 0;
    cloud_polygon->clear();
    flag = false;
    viewer->removeAllShapes();
}

void ManualPolygonClipper::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {
    // 打印调试信息，便于查看实际收到的 keySym / keyCode
    std::string ks = event.getKeySym();
    int kc = event.getKeyCode();
    qDebug() << "ManualPolygonClipper::keyboardEventOccurred keySym=" << QString::fromStdString(ks)
        << " keyCode=" << kc << " keyDown=" << event.keyDown();

    // 兼容大小写以及 keyCode（有的平台 keySym 可能为大写）
    bool isX = (ks == "x" || ks == "X" || kc == 120 || kc == 88);
    bool isEsc = (ks == "Escape" || ks == "Esc");

    if (isX && event.keyDown()) {
        isPickingMode = !isPickingMode;
        if (isPickingMode) {
            std::cout << "\n开始绘制多边形\n";
            resetPolygon();
        }
        else {
            std::cout << "\n结束绘制\n";
            projectInliers();
            viewer->removeAllShapes();
            emit clippingFinished();  // 通知裁剪完成
        }
    }
    else if (isEsc && event.keyDown()) {
        emit clippingFinished();  // ESC键取消裁剪
    }
}

void ManualPolygonClipper::mouseEventOccurred(const pcl::visualization::MouseEvent& event) {
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) {
        if (isPickingMode) {
            viewer->getRenderWindow()->MakeCurrent();

            double world_point[3];
            double displayPos[2] = { static_cast<double>(event.getX()), static_cast<double>(event.getY()) };
            getScreentPos(displayPos, world_point);
            curP = pcl::PointXYZ(world_point[0], world_point[1], world_point[2]);

            if (!flag) {
                flag = true;
            }
            else {
                char str1[512];
                sprintf(str1, "line#%03d", line_id++);
                viewer->getRenderWindow()->GetInteractor()->GetRenderWindow()->MakeCurrent();
                viewer->addLine(lastP, curP, str1);
            }
            lastP = curP;
            cloud_polygon->push_back(curP);

            // 更新点云显示
            viewer->getRenderWindow()->GetInteractor()->GetRenderWindow()->MakeCurrent();
            viewer->updatePointCloud(cloud_polygon, "polyline");

            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
            viewer->getRenderWindow()->Render();
        }
    }
}

void ManualPolygonClipper::getScreentPos(double* displayPos, double* world) {
    viewer->getRenderWindow()->MakeCurrent();

    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    double fp[4], tmp1[4], eventFPpos[4];
    renderer->GetActiveCamera()->GetFocalPoint(fp);
    fp[3] = 0.0;
    renderer->SetWorldPoint(fp);
    renderer->WorldToDisplay();
    renderer->GetDisplayPoint(tmp1);
    tmp1[0] = displayPos[0];
    tmp1[1] = displayPos[1];
    renderer->SetDisplayPoint(tmp1);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(eventFPpos);
    for (int i = 0; i < 3; i++) world[i] = eventFPpos[i];
}

int ManualPolygonClipper::inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y) {
    int i, j = poly_sides - 1, res = 0;
    for (i = 0; i < poly_sides; i++) {
        if (((poly_Y[i] < y && poly_Y[j] >= y) || (poly_Y[j] < y && poly_Y[i] >= y)) && (poly_X[i] <= x || poly_X[j] <= x)) {
            res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i]) * (poly_X[j] - poly_X[i])) < x);
        }
        j = i;
    }
    return res;
}

void ManualPolygonClipper::projectInliers() {
    vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
    double focal[3] = { 0 }, pos[3] = { 0 };
    renderer->GetActiveCamera()->GetFocalPoint(focal);
    renderer->GetActiveCamera()->GetPosition(pos);
    pcl::PointXYZ eyeLine1(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);
    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));
    pcl::PointXYZ eyeLine(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = eyeLine.x;
    coefficients->values[1] = eyeLine.y;
    coefficients->values[2] = eyeLine.z;
    coefficients->values[3] = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_Prj(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCiecle_result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_polygon);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloudCiecle_result);

    pcl::ProjectInliers<pcl::PointXYZ> projCloudIn;
    projCloudIn.setModelType(pcl::SACMODEL_PLANE);
    projCloudIn.setInputCloud(cloud_in);
    projCloudIn.setModelCoefficients(coefficients);
    projCloudIn.filter(*cloudIn_Prj);

    if (cloudCiecle_result->empty()) {
        qWarning() << "投影后的多边形点云为空!";
        return;
    }
    const size_t numPoints = cloudCiecle_result->size();

    // 使用vector替代原始数组
    std::vector<double> PloyXarr(numPoints);
    std::vector<double> PloyYarr(numPoints);

    // 安全填充数据
    for (size_t i = 0; i < numPoints; ++i) {
        PloyXarr[i] = cloudCiecle_result->points[i].x;
        PloyYarr[i] = cloudCiecle_result->points[i].y;
    }

    cloud_cliped->clear();
    // 同样修复输入点云的循环
    for (size_t i = 0; i < cloudIn_Prj->size(); ++i) {
        int ret = inOrNot1(static_cast<int>(numPoints),
            PloyXarr.data(),
            PloyYarr.data(),
            cloudIn_Prj->points[i].x,
            cloudIn_Prj->points[i].y);
        if (ret == 1) {
            cloud_cliped->points.push_back(cloud_in->points[i]);
        }
        else {
            cloud_remain->points.push_back(cloud_in->points[i]);
        }
    }

    //viewer->updatePointCloud(cloud_cliped, "clipped_result");
    //viewer->getRenderWindow()->Render();
}

// 静态事件转发
void ManualPolygonClipper::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie) {
    ManualPolygonClipper* self = static_cast<ManualPolygonClipper*>(cookie);
    if (self) self->keyboardEventOccurred(event);
}
void ManualPolygonClipper::mouseCallback(const pcl::visualization::MouseEvent& event, void* cookie) {
    ManualPolygonClipper* self = static_cast<ManualPolygonClipper*>(cookie);
    if (self) self->mouseEventOccurred(event);
}