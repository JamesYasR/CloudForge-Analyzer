#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/extract_indices.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyLine.h>
#include <vtkPolygon.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkMath.h>
#include <iostream>
#include <vector>
#include <map>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>

using namespace pcl;
using namespace std;

// 点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 2D点结构
struct Point2D {
    double x, y;
    Point2D(double _x = 0, double _y = 0) : x(_x), y(_y) {}
};

// 切割状态结构体 - 保持与原始代码相同的结构
struct CuttingState {
    vector<Point2D> polygonPoints;
    bool polygonComplete = false;
    PointCloudT::Ptr originalCloud;
    PointCloudT::Ptr insideCloud;
    PointCloudT::Ptr outsideCloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkRenderer> vtkRendererPtr;
    vtkSmartPointer<vtkRenderWindow> vtkRenderWindowPtr;
    bool showPolygon = true;
    bool cuttingMode = false;
};

// 判断点是否在多边形内（射线法） - 与原始代码相同
bool pointInPolygon(double x, double y, const vector<Point2D>& polygon) {
    int n = polygon.size();
    if (n < 3) return false;

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon[i].y > y) != (polygon[j].y > y)) &&
            (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) /
                (polygon[j].y - polygon[i].y) + polygon[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

// 计算点到平面的投影 - 与原始代码相同
void projectPointToScreen(const PointT& pt, double screenPos[2],
    vtkSmartPointer<vtkRenderer> renderer,
    vtkSmartPointer<vtkRenderWindow> renderWindow) {
    if (!renderer || !renderWindow) return;

    vtkCamera* camera = renderer->GetActiveCamera();
    int* size = renderWindow->GetSize();

    double worldPos[3] = { pt.x, pt.y, pt.z };
    double viewPos[3];

    renderer->SetWorldPoint(worldPos[0], worldPos[1], worldPos[2], 1.0);
    renderer->WorldToView();
    renderer->GetViewPoint(viewPos);

    screenPos[0] = (viewPos[0] + 1.0) * 0.5 * size[0];
    screenPos[1] = (viewPos[1] + 1.0) * 0.5 * size[1];
}

// 执行点云切割 - 与原始代码逻辑相同
void performCutting(CuttingState& state) {
    if (!state.originalCloud || state.polygonPoints.size() < 3) {
        cout << "错误: 没有点云或多边形顶点不足3个" << endl;
        return;
    }

    cout << "开始切割点云..." << endl;
    cout << "多边形顶点数: " << state.polygonPoints.size() << endl;
    cout << "原始点云点数: " << state.originalCloud->size() << endl;

    state.insideCloud = PointCloudT::Ptr(new PointCloudT);
    state.outsideCloud = PointCloudT::Ptr(new PointCloudT);

    state.insideCloud->header = state.originalCloud->header;
    state.outsideCloud->header = state.originalCloud->header;

    state.vtkRendererPtr = state.viewer->getRendererCollection()->GetFirstRenderer();
    state.vtkRenderWindowPtr = state.viewer->getRenderWindow();

    int insideCount = 0;
    for (size_t i = 0; i < state.originalCloud->size(); ++i) {
        const PointT& pt = state.originalCloud->points[i];
        double screenPos[2];
        projectPointToScreen(pt, screenPos, state.vtkRendererPtr, state.vtkRenderWindowPtr);

        if (pointInPolygon(screenPos[0], screenPos[1], state.polygonPoints)) {
            state.insideCloud->push_back(pt);
            insideCount++;
        }
        else {
            state.outsideCloud->push_back(pt);
        }
    }

    cout << "切割完成!" << endl;
    cout << "内部点数量: " << insideCount << endl;
    cout << "外部点数量: " << state.originalCloud->size() - insideCount << endl;

    state.viewer->removeAllPointClouds();

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(state.insideCloud, 255, 0, 0);
    state.viewer->addPointCloud<PointT>(state.insideCloud, red, "inside_cloud");
    state.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inside_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(state.outsideCloud, 0, 0, 255);
    state.viewer->addPointCloud<PointT>(state.outsideCloud, blue, "outside_cloud");
    state.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outside_cloud");

    pcl::io::savePCDFileBinary("PCDfiles/temp/cut/inside_points.pcd", *state.insideCloud);
    pcl::io::savePCDFileBinary("PCDfiles/temp/cut/outside_points.pcd", *state.outsideCloud);
    cout << "结果已保存为 inside_points.pcd 和 outside_points.pcd" << endl;
}

// 显示多边形 - 与原始代码逻辑相同
void showPolygonInViewer(CuttingState& state) {
    if (state.polygonPoints.size() < 2) return;

    state.viewer->removeAllShapes();

    int* size = state.vtkRenderWindowPtr->GetSize();
    if (!size || size[0] <= 0 || size[1] <= 0) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr polygonCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < state.polygonPoints.size(); ++i) {
        state.vtkRendererPtr = state.viewer->getRendererCollection()->GetFirstRenderer();
        if (!state.vtkRendererPtr) return;

        double displayPos[3];
        displayPos[0] = state.polygonPoints[i].x;
        displayPos[1] = state.polygonPoints[i].y;
        displayPos[2] = 0.0;

        double worldPos[4];
        state.vtkRendererPtr->SetDisplayPoint(displayPos);
        state.vtkRendererPtr->DisplayToWorld();
        state.vtkRendererPtr->GetWorldPoint(worldPos);

        if (worldPos[3] != 0.0) {
            polygonCloud->push_back(pcl::PointXYZ(
                worldPos[0] / worldPos[3],
                worldPos[1] / worldPos[3],
                worldPos[2] / worldPos[3]
            ));
        }
        else {
            polygonCloud->push_back(pcl::PointXYZ(0, 0, 0));
        }
    }

    if (polygonCloud->size() > 1) {
        for (size_t i = 0; i < polygonCloud->size(); ++i) {
            size_t j = (i + 1) % polygonCloud->size();
            string lineName = "polygon_line_" + to_string(i);

            state.viewer->addLine(
                polygonCloud->points[i],
                polygonCloud->points[j],
                0.0, 1.0, 0.0,
                lineName
            );

            state.viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                3,
                lineName
            );
        }
    }
}

// 更新操作提示文本 - 与原始代码逻辑相同
void updateHintText(CuttingState& state) {
    state.viewer->removeShape("hint_text");
    state.viewer->removeShape("help_text");
    state.viewer->removeShape("help_text2");
    state.viewer->removeShape("polygon_status"); 
    if (!state.cuttingMode) {
        state.viewer->addText("Press 'x' to enter manual cutting mode", 10, 30, 16, 1.0, 1.0, 1.0, "hint_text");
    }
    else {
        state.viewer->addText("Press 'x' to exit manual cutting mode", 10, 50, 16, 1.0, 1.0, 1.0, "hint_text");
        state.viewer->addText("Left click: add polygon vertex | Right click: close polygon", 10, 30, 14, 0.8, 0.8, 0.8, "help_text");
        state.viewer->addText("Press 'r' to restart |Press 'c' to cut | Press 's' to show/hide polygon", 10, 10, 14, 0.8, 0.8, 0.8, "help_text2");
        std::string polygonStatus = "Show polygon: ";
        polygonStatus += (state.showPolygon ? "ON" : "OFF");

        // 根据状态设置颜色：ON为绿色，OFF为红色
        double r = state.showPolygon ? 0.0 : 1.0;
        double g = state.showPolygon ? 1.0 : 0.0;
        double b = 0.0;

        state.viewer->addText(polygonStatus, 10, 66, 14, r, g, b, "polygon_status");
    }
}

// 键盘事件回调 - 保持与原始代码相同的逻辑
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* state_void) {
    CuttingState* state = static_cast<CuttingState*>(state_void);

    if (event.getKeySym() == "x" && event.keyDown()) {
        if (!state->cuttingMode) {
            cout << "=== 进入切割模式 ===" << endl;
            state->cuttingMode = true;
            state->polygonPoints.clear();
            state->polygonComplete = false;
            state->viewer->removeAllShapes();
            state->showPolygon = true;
        }
        else {
            cout << "=== 退出切割模式 ===" << endl;
            state->cuttingMode = false;
            state->polygonPoints.clear();
            state->polygonComplete = false;
            state->viewer->removeAllShapes();
            state->viewer->removeAllPointClouds();
            state->viewer->addPointCloud(state->originalCloud, "original_cloud");
            state->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
        }
        updateHintText(*state);
    }
    else if (event.getKeySym() == "r" && event.keyDown() && state->cuttingMode) {
        cout << "=== 重置切割模式 ===" << endl;
        state->polygonPoints.clear();
        state->polygonComplete = false;
        state->viewer->removeAllShapes();
        state->viewer->removeAllPointClouds();
        state->viewer->addPointCloud(state->originalCloud, "original_cloud");
        updateHintText(*state);
    }
    else if (event.getKeySym() == "c" && event.keyDown() && state->cuttingMode) {
        cout << "按下了 'c' 键，执行切割..." << endl;
        if (state->polygonPoints.size() >= 3) {
            performCutting(*state);
            state->polygonComplete = true;
        }
        else {
            cout << "错误: 多边形顶点不足3个" << endl;
        }
        updateHintText(*state);
    }
    else if (event.getKeySym() == "s" && event.keyDown() && state->cuttingMode) {
        cout << "按下了 's' 键，显示/隐藏多边形..." << endl;
        state->showPolygon = !state->showPolygon;
        if (!state->showPolygon) {
            state->viewer->removeAllShapes();
        }
        else {
            showPolygonInViewer(*state);
        }
        updateHintText(*state);
    }
    else if (event.getKeySym() == "q" && event.keyDown()) {
        cout << "按下了 'q' 键，退出程序..." << endl;
        state->viewer->close();
        updateHintText(*state);
    }
}

// 鼠标事件回调 - 保持与原始代码相同的逻辑
void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* state_void) {
    CuttingState* state = static_cast<CuttingState*>(state_void);

    if (!state->cuttingMode) {
        return;
    }

    if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
        event.getButton() == pcl::visualization::MouseEvent::LeftButton) {

        if (state->polygonComplete) {
            cout << "多边形已完成，按 'c' 切割或 'r' 重新开始" << endl;
            return;
        }

        int x = event.getX();
        int y = event.getY();
        state->polygonPoints.push_back(Point2D(x, y));
        cout << "添加顶点 (" << x << ", " << y << "), 当前顶点数: " << state->polygonPoints.size() << endl;

        if (state->showPolygon) {
            showPolygonInViewer(*state);
        }
        updateHintText(*state);
    }
    else if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
        event.getButton() == pcl::visualization::MouseEvent::RightButton) {

        if (state->polygonPoints.size() >= 3 && !state->polygonComplete && state->cuttingMode) {
            cout << "右键点击，自动闭合多边形..." << endl;
            state->polygonPoints.push_back(state->polygonPoints[0]);

            if (state->showPolygon) {
                showPolygonInViewer(*state);
            }

            state->polygonComplete = true;
            cout << "多边形已完成，按 'c' 切割点云" << endl;
        }
        updateHintText(*state);
    }
}

// 主功能函数 - 基于原始代码的main函数
void interactivePolygonCut(PointCloudT::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        cerr << "错误: 无效或空的点云" << endl;
        return;
    }

    cout << "=== 点云多边形切割工具 ===" << endl;
    cout << "点云大小: " << cloud->size() << " 点" << endl;

    // 初始化状态 - 模拟原始代码的全局变量
    CuttingState state;
    state.originalCloud = cloud;
    state.insideCloud = PointCloudT::Ptr(new PointCloudT);
    state.outsideCloud = PointCloudT::Ptr(new PointCloudT);

    // 创建可视化器 - 与原始代码相同
    state.viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("3D Point Cloud Polygon Cutting"));
    state.viewer->setBackgroundColor(0.1, 0.1, 0.1);
    state.viewer->addCoordinateSystem(1.0);
    state.viewer->initCameraParameters();

    // 添加点云 - 与原始代码相同
    state.viewer->addPointCloud(state.originalCloud, "original_cloud");
    state.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
    state.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "original_cloud");

    // 获取VTK渲染器和窗口
    state.vtkRendererPtr = state.viewer->getRendererCollection()->GetFirstRenderer();
    state.vtkRenderWindowPtr = state.viewer->getRenderWindow();

    // 添加初始操作提示
    updateHintText(state);

    // 注册回调函数 - 关键修改：传递state指针
    state.viewer->registerMouseCallback(mouseEventOccurred, (void*)&state);
    state.viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&state);

    // 主循环 - 与原始代码相同
    while (!state.viewer->wasStopped()) {
        state.viewer->spinOnce(100);
    }

    cout << "切割窗口已关闭" << endl;
}