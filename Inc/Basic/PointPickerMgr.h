#pragma once
#include "Basic/PointPicker.h"
#include <pcl/point_types.h>
#include <QObject>

class PointPickerMgr : public QObject {
    Q_OBJECT
public:
    explicit PointPickerMgr(vtkRenderWindowInteractor* interactor,
        int numPoints,
        QObject* parent = nullptr);
    ~PointPickerMgr() override = default;

    // 阻塞式一次性接口：返回所有选到的点
    std::vector<std::array<double, 3>> GetPickedPoints() const { return pickedPoints_; }
    std::vector<pcl::PointXYZ> GetPickedPCLPoints() const;

signals:
    // 非阻塞式也可选：选完触发
    void selectionCompleted(bool success);

private:
    std::vector<std::array<double, 3>> pickedPoints_;
    PointPicker* picker_ = nullptr;
};