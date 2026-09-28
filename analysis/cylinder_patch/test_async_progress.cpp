// test_async_progress.cpp -- 验证"后台计算 + 定时器轮询进度"机制
// 核心断言: 工作线程计算期间, GUI 线程的定时器仍在持续触发(即事件循环未被阻塞)
#include <QCoreApplication>
#include <QTimer>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

struct State {
    std::atomic<bool> cancelRequested{ false };
    std::atomic<int> current{ 0 };
    std::atomic<int> total{ 0 };
    std::mutex mtx;
    std::string stage;
};

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const bool doCancel = (argc > 1 && std::string(argv[1]) == "cancel");

    auto state = std::make_shared<State>();
    int ticks = 0;
    int lastShownPct = -1;
    auto t0 = std::chrono::steady_clock::now();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        ++ticks;   // GUI 线程每 100ms 跳一次: 证明事件循环活着
        const int cur = state->current.load();
        const int tot = state->total.load();
        if (tot > 0) {
            const int pct = 100 * cur / tot;
            if (pct / 20 != lastShownPct / 20) {
                lastShownPct = pct;
                std::string stage;
                { std::lock_guard<std::mutex> lk(state->mtx); stage = state->stage; }
                std::printf("  [GUI 定时器] tick=%d 进度=%d%% (%d/%d) 阶段=%s\n",
                            ticks, pct, cur, tot, stage.c_str());
            }
        }
    });
    timer.start(100);

    // 工作线程: 模拟 5 秒的重计算, 每 50ms 汇报一次进度
    QFutureWatcher<void> watcher;
    QObject::connect(&watcher, &QFutureWatcher<void>::finished, [&]() {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        std::printf("\n[结果] 计算耗时 %lld ms, 期间 GUI 定时器触发 %d 次\n",
                    (long long)ms, ticks);
        std::printf("[判定] 事件循环是否始终存活: %s (旧同步写法下应为 0 次)\n",
                    ticks > 20 ? "是 ✓" : "否 ✗");
        std::printf("[判定] 取消是否生效: %s\n",
                    state->cancelRequested.load() ? "已请求" : "未请求");
        app.quit();
    });

    watcher.setFuture(QtConcurrent::run([state, doCancel]() {
        const int totalSteps = 100;
        state->total.store(totalSteps);
        { std::lock_guard<std::mutex> lk(state->mtx); state->stage = "模拟重计算"; }
        for (int i = 0; i < totalSteps; ++i) {
            if (i == 20 && doCancel) {
                std::printf("  [工作线程] 第 %d 步模拟用户取消\n", i);
                state->cancelRequested.store(true);
            }
            if (state->cancelRequested.load()) {
                std::printf("  [工作线程] 在检查点中止于第 %d 步\n", i);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            state->current.store(i + 1);
        }
    }));

    return app.exec();
}
