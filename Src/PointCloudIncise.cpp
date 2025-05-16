#include "PointCloudIncise.h"
#include <QDebug>
#include <vtkWorldPointPicker.h>
bool isMouseCallbackRegistered = false;

PointCloudClipper::PointCloudClipper(QObject* parent)
    : QObject(parent), currentState(IDLE) {
}

void PointCloudClipper::setupClipper(pcl::visualization::PCLVisualizer::Ptr viewer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud,
    vtkRenderWindow* renderWindow) {
    this->viewer = viewer;
    this->originalCloud = originCloud;
    this->renderWindow = renderWindow;

    // 注册事件回调
    if (!isMouseCallbackRegistered) {
        viewer->registerMouseCallback([this](const auto& event) {
            /*qDebug() << "Mouse Event Received:" << event.getType();*/
            this->mouseEvent(event);
            });
        isMouseCallbackRegistered = true;
    }
}

void PointCloudClipper::startClipping() {
    currentState = DRAWING;
    polygonPoints.clear();
    emit updateButtonStates(true);
    viewer->addText("Crop mode: Click to draw polygon, right-click to undo", 10, 20, "clipping_status");

    // 获取摄像头参数并设置裁切平面
    vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
    vtkCamera* camera = renderer->GetActiveCamera();
    double focalPoint[3];
    camera->GetFocalPoint(focalPoint);
    double viewDirection[3];
    camera->GetDirectionOfProjection(viewDirection);

    planeNormal = Eigen::Vector3f(viewDirection[0], viewDirection[1], viewDirection[2]);
    planeOrigin = Eigen::Vector3f(focalPoint[0], focalPoint[1], focalPoint[2]);
    computeLocalCoordinateSystem(); // 计算局部坐标系

    renderWindow->Render();
}

void PointCloudClipper::cancelClipping() {
    currentState = IDLE;
    clearTempVisuals();
    emit updateButtonStates(false);
    renderWindow->Render();
}

void PointCloudClipper::applyClipping() {
    if (polygonPoints.size() < 3) {
        emit clippingFinished(false);
        return;
    }

    // 执行裁剪逻辑
    clippedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : *originalCloud) {
        if (pointInPolygon(point)) {
            clippedCloud->push_back(point);
        }
    }

    // 替换原始点云
    *originalCloud = *clippedCloud;
    viewer->updatePointCloud(originalCloud, "cloud");

    currentState = IDLE;
    clearTempVisuals();
    emit clippingFinished(true);
    renderWindow->Render();
}

// 关键事件处理
void PointCloudClipper::mouseEvent(const pcl::visualization::MouseEvent& event) {
    if (currentState != DRAWING) return;
    renderWindow->MakeCurrent();  // 激活上下文
    if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
        event.getButton() == pcl::visualization::MouseEvent::LeftButton) {

        // 坐标转换
        /*double worldPos[3];
        convertScreenToWorld(event.getX(), event.getY(), worldPos);*/
        double worldPos[4];
        convertScreenToWorld(event.getX(), event.getY(), worldPos);

        // 存储多边形点
        pcl::PointXYZ pt(worldPos[0], worldPos[1], worldPos[2]);
        polygonPoints.push_back(pt);

        // 存储前三个坐标

        // 更新可视化
        updateVisualization();
    }

    if (event.getButton() == pcl::visualization::MouseEvent::RightButton &&
        !polygonPoints.empty()) {
        polygonPoints.pop_back(); // 移除最后一个点
        updateVisualization();
    }
    renderWindow->Render();  // 确保立即渲染
}

// 多边形包含性检测（射线法）
bool PointCloudClipper::pointInPolygon(const pcl::PointXYZ& point) {
    pcl::PointXYZ projPoint = projectPointToPlane(point);
    Eigen::Vector2f localCoords = projectToLocal(projPoint);

    // 转换多边形顶点到局部坐标
    std::vector<Eigen::Vector2f> polygon2D;
    for (const auto& p : polygonPoints) {
        pcl::PointXYZ pp(p.x, p.y, p.z);
        Eigen::Vector2f pc = projectToLocal(pp);
        polygon2D.push_back(pc);
    }

    // 射线法检测
    int crossings = 0;
    int n = polygon2D.size();
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2f& p1 = polygon2D[i];
        const Eigen::Vector2f& p2 = polygon2D[(i + 1) % n];
        if ((p1.y() > localCoords.y()) != (p2.y() > localCoords.y())) {
            float xIntersect = (localCoords.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
            if (localCoords.x() < xIntersect) crossings++;
        }
    }
    return (crossings % 2) == 1;
}

// 在PointCloudIncise.cpp中添加以下实现

void PointCloudClipper::convertScreenToWorld(int x, int y, double* worldPos) {
    renderWindow->MakeCurrent();
    vtkRendererCollection* renderers = renderWindow->GetRenderers();
    if (!renderers) return;
    vtkRenderer* renderer = renderers->GetFirstRenderer();
    vtkCamera* camera = renderer->GetActiveCamera();

    // 获取屏幕坐标（Y轴翻转）
    int* size = renderWindow->GetSize();
    double displayPos[3] = {
        static_cast<double>(x),
        static_cast<double>(size[1] - y - 1),
        0.0
    };

    // 计算近远点生成射线（使用4维数组避免栈溢出）
    double worldNear[4], worldFar[4]; // VTK需要4维齐次坐标
    vtkInteractorObserver::ComputeDisplayToWorld(renderer, displayPos[0], displayPos[1], 0.0, worldNear);
    vtkInteractorObserver::ComputeDisplayToWorld(renderer, displayPos[0], displayPos[1], 1.0, worldFar);

    // 获取摄像机参数
    double cameraPos[3], focalPoint[3], viewDirection[3];
    camera->GetPosition(cameraPos);
    camera->GetFocalPoint(focalPoint);

    // 计算视线方向（摄像机看向焦点的方向）
    vtkMath::Subtract(focalPoint, cameraPos, viewDirection);
    vtkMath::Normalize(viewDirection);

    // 构造动态投影平面（通过焦点且垂直于视线）
    const double planeNormal[3] = { viewDirection[0], viewDirection[1], viewDirection[2] };
    const double planeOrigin[3] = { focalPoint[0], focalPoint[1], focalPoint[2] };

    // 计算射线方向
    double rayDirection[3] = {
        worldFar[0] - worldNear[0],
        worldFar[1] - worldNear[1],
        worldFar[2] - worldNear[2]
    };

    // 求解射线与平面交点
    double denominator = vtkMath::Dot(planeNormal, rayDirection);
    if (fabs(denominator) < 1e-6) {
        qDebug() << "cant pick a piont:parell";
        return;
    }

    double t = (vtkMath::Dot(planeNormal, planeOrigin) - vtkMath::Dot(planeNormal, worldNear)) / denominator;
    worldPos[0] = worldNear[0] + t * rayDirection[0];
    worldPos[1] = worldNear[1] + t * rayDirection[1];
    worldPos[2] = worldNear[2] + t * rayDirection[2];

}

void PointCloudClipper::updateVisualization() {
    static std::vector<std::string> lineIDs;

    // 清除旧线段
    for (const auto& id : lineIDs) {
        viewer->removeShape(id);
    }
    lineIDs.clear();

    // 绘制新多边形
    if (polygonPoints.size() >= 2) {
        for (size_t i = 0; i < polygonPoints.size(); ++i) {
            const auto& p1 = polygonPoints[i];
            const auto& p2 = polygonPoints[(i + 1) % polygonPoints.size()];

            std::string lineId = "polygon_line_" + std::to_string(i);
            viewer->addLine<pcl::PointXYZ>(p1, p2, 1.0, 0.0, 0.0, lineId);
            lineIDs.push_back(lineId);
        }
    }

    // 更新操作提示
    viewer->updateText("当前点数: " + std::to_string(polygonPoints.size()), 10, 60, "polygon_count");
    renderWindow->Render();
}

// 清除临时图形
void PointCloudClipper::clearTempVisuals() {
    // 清除所有多边形线段
    for (int i = 0; i < 50; ++i) { // 假设最多50条线段
        viewer->removeShape("polygon_line_" + std::to_string(i));
    }

    // 清除提示文本
    viewer->removeText3D("clipping_status");
    viewer->removeText3D("polygon_count");
    renderWindow->Render();
}

// 完整的键盘事件处理
void PointCloudClipper::keyboardEvent(const pcl::visualization::KeyboardEvent& event) {
    renderWindow->MakeCurrent();  // 激活上下文
    if (event.getKeySym() == "Escape" && event.keyDown()) {
        cancelClipping();
        return;
    }

    // 支持退格键删除最后一点
    if (currentState == DRAWING && event.getKeySym() == "BackSpace" && event.keyDown()) {
        if (!polygonPoints.empty()) {
            polygonPoints.pop_back();
            updateVisualization();
        }
    }
    renderWindow->Render();
}

bool PointCloudClipper::isClippingActive() const {
    return currentState != ClippingState::IDLE;
}
bool PointCloudClipper::polygonEmpty() const {
    return polygonPoints.empty();
}

// 投影点到裁切平面
pcl::PointXYZ PointCloudClipper::projectPointToPlane(const pcl::PointXYZ& point) {
    Eigen::Vector3f p(point.x, point.y, point.z);
    Eigen::Vector3f o = planeOrigin;
    Eigen::Vector3f n = planeNormal;
    float t = (p - o).dot(n);
    Eigen::Vector3f proj = p - t * n;
    return pcl::PointXYZ(proj.x(), proj.y(), proj.z());
}

// 转换到局部二维坐标系
Eigen::Vector2f PointCloudClipper::projectToLocal(const pcl::PointXYZ& projPoint) {
    Eigen::Vector3f p(projPoint.x - planeOrigin.x(),
                      projPoint.y - planeOrigin.y(),
                      projPoint.z - planeOrigin.z());
    float u = p.dot(localU);
    float v = p.dot(localV);
    return Eigen::Vector2f(u, v);
}

// 计算局部坐标系
void PointCloudClipper::computeLocalCoordinateSystem() {
    Eigen::Vector3f n = planeNormal.normalized();
    Eigen::Vector3f tempAxis = (n == Eigen::Vector3f::UnitZ()) ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitZ();
    localU = n.cross(tempAxis).normalized();
    localV = n.cross(localU).normalized();
}