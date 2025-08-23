#include "Basic/PointPicker.h"
#include <vtkPointPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkInteractorStyle.h>
#include <QDebug>
#include <QEventLoop>
#include <QTimer>

PointPicker::PointPicker(vtkRenderWindowInteractor* interactor, int numPoints, QObject* parent)
    : QObject(parent), interactor(interactor), targetPoints(numPoints)
{
    // 创建回调对象
    callback = vtkSmartPointer<PickerCallback>::New();
    callback->SetPicker(this);

    // 注册事件监听
    interactor->AddObserver(vtkCommand::LeftButtonPressEvent, callback);
    interactor->AddObserver(vtkCommand::KeyPressEvent, callback);

    // 设置超时定时器 (30秒无操作超时)
    timeoutTimer.setSingleShot(true);
    timeoutTimer.setInterval(30000);
    connect(&timeoutTimer, &QTimer::timeout, this, [this]() {
        cancelled = true;
        qDebug() << "点选择超时";
        emit selectionCompleted(false);
        if (eventLoop.isRunning()) eventLoop.quit();
        });
}

PointPicker::~PointPicker()
{
    // 清理临时可视化对象
    if (interactor && interactor->GetRenderWindow()) {
        if (auto renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()) {
            for (auto& actor : pointActors) {
                renderer->RemoveActor(actor);
            }
        }
        interactor->Render();
    }

    // 移除观察者
    if (interactor && callback) {
        interactor->RemoveObserver(callback);
    }
}

void PointPicker::StartSelection()
{
    // 重置状态
    completed = false;
    cancelled = false;
    selectedPoints.clear();

    // 保存原始交互样式
    originalObserver = interactor->GetInteractorStyle();

    // 设置点拾取器
    interactor->SetPicker(vtkSmartPointer<vtkPointPicker>::New());

    // 启动超时定时器
    timeoutTimer.start();

    // 启动事件循环（非阻塞）
    if (!cancelled) {
        eventLoop.exec(); // 进入事件循环
    }
}

void PointPicker::CancelSelection()
{
    cancelled = true;
    if (eventLoop.isRunning()) {
        eventLoop.quit();
    }
}

const std::vector<std::array<double, 3>>& PointPicker::GetSelectedPoints() const
{
    return selectedPoints;
}

void PointPicker::HandlePointPick(double pos[3])
{
    // 保存选择的点
    selectedPoints.push_back({ pos[0], pos[1], pos[2] });

    // 可视化点
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(0.5);
    sphere->SetCenter(pos);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1, 0, 0); // 红色

    if (auto renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()) {
        renderer->AddActor(actor);
        pointActors.push_back(actor);
        interactor->Render();
    }

    qDebug() << "已选择点:" << pos[0] << pos[1] << pos[2];

    // 检查是否达到目标点数
    if (targetPoints > 0 && selectedPoints.size() >= static_cast<size_t>(targetPoints)) {
        completed = true;
        timeoutTimer.stop();
        emit selectionCompleted(true);
        if (eventLoop.isRunning()) eventLoop.quit();
    }
}

void PointPicker::HandleKeyPress(const std::string& key)
{
    if (key == "Return") {
        completed = true;
        timeoutTimer.stop();
        emit selectionCompleted(true);
        if (eventLoop.isRunning()) eventLoop.quit();
    }
    else if (key == "Escape") {
        cancelled = true;
        timeoutTimer.stop();
        emit selectionCompleted(false);
        if (eventLoop.isRunning()) eventLoop.quit();
    }
}

void PointPicker::PickerCallback::Execute(vtkObject* caller, unsigned long eventId, void* callData)
{
    vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    if (!iren || !picker) return;

    switch (eventId) {
    case vtkCommand::LeftButtonPressEvent: {
        // 获取鼠标位置
        int x = iren->GetEventPosition()[0];
        int y = iren->GetEventPosition()[1];

        // 使用点拾取器
        vtkPointPicker* pointPicker = vtkPointPicker::SafeDownCast(iren->GetPicker());
        if (pointPicker && pointPicker->Pick(x, y, 0, iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer())) {
            double pos[3];
            pointPicker->GetPickPosition(pos);
            picker->HandlePointPick(pos);
        }
        break;
    }
    case vtkCommand::KeyPressEvent: {
        std::string key = iren->GetKeySym();
        picker->HandleKeyPress(key);
        break;
    }
    }
}