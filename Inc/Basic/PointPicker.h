#ifndef POINTPICKER_H
#define POINTPICKER_H

#include <QObject>
#include <vector>
#include <array>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkPointPicker.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <QEventLoop>
#include <QTimer>

class PointPicker : public QObject
{
    Q_OBJECT
public:
    explicit PointPicker(vtkRenderWindowInteractor* interactor, int numPoints, QObject* parent = nullptr);
    ~PointPicker();

    void StartSelection(); // 非阻塞启动点选择
    void CancelSelection(); // 取消选择

    const std::vector<std::array<double, 3>>& GetSelectedPoints() const;
    void HandlePointPick(double pos[3]);

    QEventLoop eventLoop; // 用于非阻塞等待的事件循环
signals:
    void selectionCompleted(bool success); // 选择完成信号

private:
    
    void HandleKeyPress(const std::string& key);

    // 内部回调类
    class PickerCallback : public vtkCommand
    {
    public:
        static PickerCallback* New() { return new PickerCallback; }
        void SetPicker(PointPicker* picker) { this->picker = picker; }
        virtual void Execute(vtkObject* caller, unsigned long eventId, void* callData) override;

    private:
        PointPicker* picker = nullptr;
    };

    vtkRenderWindowInteractor* interactor = nullptr;
    vtkSmartPointer<vtkInteractorObserver> originalObserver; // 保存原始的交互器样式
    int targetPoints; // 需要选择的点数
    std::vector<std::array<double, 3>> selectedPoints; // 保存选择的点坐标
    std::vector<vtkSmartPointer<vtkActor>> pointActors; // 可视化点的actor
    vtkSmartPointer<PickerCallback> callback; // 事件回调

    QTimer timeoutTimer;  // 超时定时器
    bool completed = false; // 是否完成选择
    bool cancelled = false; // 是否取消选择
};

#endif // POINTPICKER_H