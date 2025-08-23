#include "Basic/PointPickerMgr.h"
#include <QEventLoop>

PointPickerMgr::PointPickerMgr(vtkRenderWindowInteractor* interactor,
    int numPoints,
    QObject* parent)
    : QObject(parent)
{
    picker_ = new PointPicker(interactor, numPoints, this);

    // 用局部事件循环把“异步信号”变“阻塞”
    QEventLoop loop;
    connect(picker_, &PointPicker::selectionCompleted,
        this, [&](bool ok) {
            if (ok)
                pickedPoints_ = picker_->GetSelectedPoints();
            loop.quit();  // 确保退出循环
        });

    picker_->StartSelection(); // 启动点选择
    loop.exec();  // 阻塞直到选择完成
}

std::vector<pcl::PointXYZ> PointPickerMgr::GetPickedPCLPoints() const
{
    std::vector<pcl::PointXYZ> out;
    out.reserve(pickedPoints_.size());
    for (const auto& p : pickedPoints_) {
        out.emplace_back(static_cast<float>(p[0]),
            static_cast<float>(p[1]),
            static_cast<float>(p[2]));
    }
    return out;   // NRVO / 移动构造无额外拷贝
}