// 聚焦两个缺陷：真实 STL 导入入口，以及窗口关闭与 worker 生命周期。
// 本验证单独编译，-fno-access-control 仅用于注入可控的后台工作，不修改产品 API。
#include "CloudForgeAnalyzer.h"
#include <QApplication>
#include <QClipboard>
#include <QDataStream>
#include <QElapsedTimer>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QTemporaryDir>
#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

static int failures = 0;
static void check(bool value, const char* name)
{
    std::cout << (value ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (!value) ++failures;
}
template<class F> static bool waitFor(F ready, int timeout = 8000)
{
    QElapsedTimer elapsed; elapsed.start();
    while (!ready() && elapsed.elapsed() < timeout) {
        QApplication::processEvents();
        QThread::msleep(2);
    }
    return ready();
}

static void writeTriangle(const QString& path, bool binary)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) throw std::runtime_error("fixture open failed");
    if (binary) {
        file.write(QByteArray(80, ' '));
        QDataStream stream(&file);
        stream.setByteOrder(QDataStream::LittleEndian);
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        stream << quint32(1);
        for (float value : {0.f,0.f,1.f, 0.f,0.f,0.f, 1.f,0.f,0.f, 0.f,1.f,0.f}) stream << value;
        stream << quint16(0);
    } else {
        file.write("solid triangle\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\n"
                   "vertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid triangle\n");
    }
}

static void importTest(CloudForgeAnalyzer& window, const QString& path, bool valid)
{
    const auto before = window.CloudMap;
    bool selected = false, parameters = false, result = false, error = false;
    QTimer drive;
    QObject::connect(&drive, &QTimer::timeout, &window, [&]() {
        for (auto* widget : QApplication::topLevelWidgets()) {
            if (!widget->isVisible()) continue;
            if (auto* file = qobject_cast<QFileDialog*>(widget)) {
                if (!selected) {
                    selected = true;
                    file->selectFile(path);
                    QMetaObject::invokeMethod(file, "accept", Qt::QueuedConnection);
                }
            } else if (auto* message = qobject_cast<QMessageBox*>(widget)) {
                result |= message->windowTitle() == QStringLiteral("转换完成");
                error |= message->windowTitle() == QStringLiteral("错误");
                message->accept();
            } else if (auto* dialog = qobject_cast<QDialog*>(widget)) {
                if (dialog->windowTitle().startsWith(QStringLiteral("这是一个STL文件"))) {
                    parameters = true;
                    // 直接使用顶点，两个文件格式均应得到相同的三个坐标。
                    for (auto* box : dialog->findChildren<QCheckBox*>()) box->setChecked(false);
                    for (auto* button : dialog->findChildren<QPushButton*>()) {
                        if (button->text() == QStringLiteral("转换")) { button->click(); break; }
                    }
                }
            }
        }
    });
    drive.start(10);
    QMetaObject::invokeMethod(&window, "Slot_fi_openSTL_Triggered", Qt::QueuedConnection);
    check(waitFor([&]() {
        return !window.m_asyncRunning && (valid ? result && window.CloudMap != before : error);
    }), valid ? "Unicode STL import finishes through the real dialogs" : "invalid STL reports failure");
    drive.stop();
    if (valid) {
        const auto& clouds = window.CloudMap;
        bool coordinates = clouds.size() == 1;
        std::vector<std::array<float, 3>> actual;
        if (coordinates) {
            for (const auto& p : clouds.begin()->second->points) actual.push_back({p.x,p.y,p.z});
        }
        std::sort(actual.begin(), actual.end());
        const std::vector<std::array<float,3>> expected{{0,0,0},{0,1,0},{1,0,0}};
        check(selected && parameters && result && !error && actual == expected,
              "ASCII/binary import preserves exact triangle coordinates");
    } else {
        check(!parameters && !result && window.CloudMap == before,
              "failed read opens no conversion dialog and leaves scene intact");
    }
}

static void closeTest(bool cooperative, bool throws)
{
    CloudForgeAnalyzer window;
    window.show();
    std::atomic<bool> started{false}, release{false}, stopped{false};
    bool published = false, extraWork = false;
    auto work = [&]() {
        started = true;
        while (!release.load()) {
            if (cooperative && !window.PostProgress(0, 0, "计算")) break;
            QThread::msleep(1);
        }
        stopped = true;
        if (throws) throw std::runtime_error("injected worker failure during shutdown");
    };
    window.RunAsyncVoid("计算", work, [&]() { published = true; });
    check(waitFor([&]() { return started.load(); }), "worker starts before closing");
    const bool closed = window.close();
    check(!closed && window.m_shuttingDown && window.m_asyncState->cancelRequested,
          "close immediately requests cancellation and defers window destruction");
    window.RunAsyncVoid("禁止重入", [&]() { extraWork = true; });
    if (!cooperative) {
        check(window.isVisible() && window.windowTitle() == QStringLiteral("正在停止计算"),
              "uninterruptible stage stays visible while stopping");
        check(!window.close(), "repeated close cannot destroy a running worker");
        QApplication::processEvents();
        // QWidget::grab 可在 Wayland/XWayland 桌面捕获控件，避免系统屏幕抓取返回黑图。
        window.grab().save("build/shutdown-waiting.png");
        release = true;
    }
    check(waitFor([&]() { return !window.isVisible(); }) && stopped && !window.m_asyncRunning,
          "window actually closes only after the worker exits");
    check(!published && !extraWork, "shutdown publishes no result and starts no follow-up task");
}

static void aboutTest(CloudForgeAnalyzer& window)
{
    bool opened = false, copied = false;
    QTimer::singleShot(0, &window, [&]() {
        auto* dialog = window.findChild<QDialog*>(QStringLiteral("aboutDialog"));
        if (!dialog) return;
        opened = dialog->isVisible();
        dialog->grab().save("build/about-master.png");
        for (auto* button : dialog->findChildren<QPushButton*>()) {
            if (button->text() == QStringLiteral("复制版本信息")) button->click();
        }
        copied = QApplication::clipboard()->text().contains(QStringLiteral("CloudForge Analyzer"));
        dialog->accept();
    });
    window.ui->action_about->trigger();
    check(opened && copied, "master help menu opens About and copies version information");
}

int main(int argc, char** argv)
{
    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QApplication app(argc, argv);
    if (argc > 1 && std::string(argv[1]) == "exit") {
        // 使用真实 app.exec()，验证最后一个窗口关闭后进程确实返回。
        CloudForgeAnalyzer window;
        window.show();
        std::atomic<bool> stopped{false};
        bool published = false;
        window.RunAsyncVoid("计算", [&]() {
            while (window.PostProgress(0,0,"计算")) QThread::msleep(1);
            stopped = true;
        }, [&]() { published = true; });
        QTimer::singleShot(100, &window, [&]() { window.close(); });
        QTimer::singleShot(5000, &app, [&]() { ++failures; app.quit(); });
        app.exec();
        check(stopped && !published && !window.m_asyncRunning, "real event loop exits without background computation");
        return failures ? 1 : 0;
    }
    app.setQuitOnLastWindowClosed(false);
    {
        QTemporaryDir dir;
        const QString folder = dir.path() + QStringLiteral("/中文路径 零件𠀀");
        QDir().mkpath(folder);
        CloudForgeAnalyzer window;
        window.show();
        aboutTest(window);
        for (bool binary : {false,true}) {
            const QString path = folder + (binary ? QStringLiteral("/二进制零件.STL") : QStringLiteral("/文本零件.stl"));
            writeTriangle(path, binary);
            importTest(window, path, true);
        }
        const QString invalid = folder + QStringLiteral("/损坏模型.stl");
        QFile file(invalid); file.open(QIODevice::WriteOnly); file.write("not a mesh"); file.close();
        importTest(window, invalid, false);
        window.close();
    }
    closeTest(true, false);
    closeTest(false, true);
    std::cout << "[SUMMARY] failures=" << failures << std::endl;
    return failures ? 1 : 0;
}
