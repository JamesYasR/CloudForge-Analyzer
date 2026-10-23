#pragma once
#include "Application/TaskRunner.h"
#include "Basic/PolygonClipTool.h"
#include "Scene/CloudScene.h"
#include <QObject>
#include <optional>

class ClipController : public QObject {
public:
    // 兼容窗口通过这组窄接口暴露场景，控制器不访问旧 maps 或 PCL actor 表。
    struct Host {
        std::function<bool()> busy;
        std::function<void()> prepareInteraction;
        std::function<std::vector<PolygonClipTool::Target>()> targets;
        std::function<CloudScene::Snapshot()> read;
        std::function<bool(const CloudScene::Edit&)> commit;
        std::function<CloudScene::Id()> allocateId;
        std::function<void(TaskRunner::Work, TaskRunner::Finish)> run;
        std::function<void(const QString&)> message;
    };
    ClipController(PolygonClipTool& tool, Host host, QObject* parent = nullptr);
    void toggle();
    void preview();
    void apply();
private:
    PolygonClipTool& tool_;
    Host host_;
    std::optional<CloudScene::Record> previewSource_;
    std::uint64_t generation_ = 0;
};
