#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// 独立于窗口的工作线程状态。关闭窗口后仍由任务持有，不能捕获窗口 this。
struct TaskContext {
    std::atomic<bool> cancelRequested{false};
    std::atomic<int> current{0};
    std::atomic<int> total{0};
    std::mutex mtx;
    std::string stage;
    std::vector<std::string> pendingLog;

    bool report(int value, int count, const std::string& action) {
        current.store(value);
        total.store(count);
        if (!action.empty()) { std::lock_guard<std::mutex> lock(mtx); stage = action; }
        return !cancelRequested.load();
    }
};
