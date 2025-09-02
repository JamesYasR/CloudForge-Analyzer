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
    vtkSmartPointer<vtkRenderWindowInteractor> mainInteractor,
    pcl::visualization::PCLVisualizer*  mainviewer,
    QObject* parent
) : QObject(parent)
, cloud_in(input_cloud)
, cloud_polygon(new pcl::PointCloud<pcl::PointXYZ>)
, cloud_cliped(new pcl::PointCloud<pcl::PointXYZ>)
, flag(false)
, isPickingMode(false)
, line_id(0)
,viewer(mainviewer)
{
    // 保存原始交互器样式
    //originalStyle = mainInteractor->GetInteractorStyle();
    this->mainInteractor = mainInteractor;
    originalStyle = vtkInteractorStyle::SafeDownCast(mainInteractor->GetInteractorStyle());

    // 确保指针有效
    if (!originalStyle) {
        qWarning() << "无法获取原始交互器样式";
        originalStyle = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
    }

    // 设置自定义交互模式
    vtkNew<vtkInteractorStyleSwitch> styleSwitch;
    styleSwitch->SetCurrentStyleToTrackballCamera();
    mainInteractor->SetInteractorStyle(styleSwitch);

    viewer->setupInteractor(mainInteractor,mainRenderWindow);

    viewer->addPointCloud(cloud_polygon, "polyline");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polyline");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "polyline");
    viewer->addPointCloud(cloud_in, "cloud_in");

    viewer->registerKeyboardCallback(&ManualPolygonClipper::keyboardCallback, this);
    viewer->registerMouseCallback(&ManualPolygonClipper::mouseCallback, this);

}

ManualPolygonClipper::~ManualPolygonClipper() {
    cleanup();
}

void ManualPolygonClipper::cleanup() {
    if (mainInteractor && originalStyle) {
        // 恢复原始交互器样式
        mainInteractor->SetInteractorStyle(originalStyle);

        // 重新设置 PCLVisualizer 的交互器
        if (viewer) {
            viewer->setupInteractor(mainInteractor, mainInteractor->GetRenderWindow());
        }

        // 确保交互器启用
        if (!mainInteractor->GetEnabled()) {
            mainInteractor->Enable();
        }

        // 刷新渲染
        mainInteractor->Render();
    }

    if (viewer) {
        // 注销回调函数
        viewer->registerMouseCallback(nullptr, nullptr);
        viewer->registerKeyboardCallback(nullptr, nullptr);

        // 移除临时对象
        viewer->removePointCloud("polyline");
        viewer->removePointCloud("aftercut");

        // 移除所有线段
        for (int i = 0; i < line_id; ++i) {
            char str[512];
            sprintf(str, "line#%03d", i);
            viewer->removeShape(str);
        }
    }
}
void ManualPolygonClipper::startClipping() {
    resetPolygon();
    isPickingMode = true;
    viewer->getRenderWindow()->Render();  // 触发渲染
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ManualPolygonClipper::getClippedCloud() const {
    return cloud_cliped;
}

void ManualPolygonClipper::resetPolygon() {
    line_id = 0;
    cloud_polygon->clear();
    flag = false;
    viewer->removeAllShapes();
}

void ManualPolygonClipper::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {
    if (event.getKeySym() == "x" && event.keyDown()) {
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
    else if (event.getKeySym() == "Escape" && event.keyDown()) {
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
    }

    //delete[] PloyXarr;
    //delete[] PloyYarr;

    viewer->updatePointCloud(cloud_cliped, "clipped_result");
    viewer->getRenderWindow()->Render();
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