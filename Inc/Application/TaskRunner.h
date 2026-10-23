#pragma once
#include "Application/TaskContext.h"
#include <QObject>
#include <QFuture>

// GUI 线程启动和收尾；仅计算闭包进入工作线程。试点使用，其他入口暂不迁移。
class TaskRunner : public QObject {
public:
    enum class Status { Success, Cancelled, Failed };
    using Work = std::function<void(const std::shared_ptr<TaskContext>&)>;
    using Finish = std::function<void(Status)>;
    explicit TaskRunner(QObject* parent = nullptr) : QObject(parent) {}
    ~TaskRunner() override;
    bool start(std::shared_ptr<TaskContext> context, Work work, Finish finish);
    void shutdown();
private:
    QFuture<void> future_;
    std::shared_ptr<TaskContext> context_;
    bool running_ = false;
    bool closing_ = false;
};
