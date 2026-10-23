#include "Application/TaskRunner.h"
#include <QFutureWatcher>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>

TaskRunner::~TaskRunner() { shutdown(); }

bool TaskRunner::start(std::shared_ptr<TaskContext> context, Work work, Finish finish)
{
    Q_ASSERT(QThread::currentThread() == thread());
    if (running_ || closing_ || !context || !work) return false;
    auto status = std::make_shared<Status>(Status::Failed);
    auto watcher = new QFutureWatcher<void>(this);
    context_ = context;
    running_ = true;
    connect(watcher, &QFutureWatcher<void>::finished, this, [this, watcher, context, status, finish]() {
        watcher->deleteLater();
        running_ = false;
        context_.reset();
        if (!closing_ && finish) finish(context->cancelRequested.load() ? Status::Cancelled : *status);
    });
    try {
        future_ = QtConcurrent::run([context, work = std::move(work), status]() {
            try {
                if (!context->cancelRequested.load()) work(context);
                *status = context->cancelRequested.load() ? Status::Cancelled : Status::Success;
            } catch (...) {
                *status = Status::Failed;
            }
        });
        watcher->setFuture(future_);
    } catch (...) {
        context->cancelRequested.store(true);
        future_.waitForFinished();
        running_ = false;
        context_.reset();
        delete watcher;
        return false;
    }
    return true;
}

void TaskRunner::shutdown()
{
    closing_ = true;
    if (context_) context_->cancelRequested.store(true);
    future_.waitForFinished();
}
