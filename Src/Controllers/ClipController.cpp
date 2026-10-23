#include "Controllers/ClipController.h"
#include <utility>

ClipController::ClipController(PolygonClipTool& tool, Host host, QObject* parent)
    : QObject(parent), tool_(tool), host_(std::move(host))
{
    connect(&tool_, &PolygonClipTool::activeChanged, this, [this](bool active) { if (!active) previewSource_.reset(); });
}

void ClipController::toggle()
{
    if (tool_.isActive()) { tool_.stop(); return; }
    if (host_.busy()) { host_.message("计算进行中，请稍后裁切。"); return; }
    host_.prepareInteraction();
    if (!tool_.start(host_.targets())) host_.message("不可裁切: 当前没有可选取的点云");
}

void ClipController::preview()
{
    if (host_.busy() || !tool_.isActive()) return;
    previewSource_.reset();
    const auto scene = host_.read();
    const auto source = scene.records.find(tool_.target().name);
    if (source == scene.records.end() || source->second.cloud != tool_.target().cloud) { tool_.stop(); return; }
    const auto record = source->second;
    const auto request = tool_.request();
    const auto ticket = tool_.ticket();
    auto result = std::make_shared<CloudClipper::Result>();
    host_.run([request, result](const std::shared_ptr<TaskContext>& context) {
        *result = CloudClipper::compute(request, [context](int value, int total, const std::string& action) {
            return context->report(value, total, action);
        });
    }, [this, result, ticket, record, generation = scene.generation](TaskRunner::Status status) {
        if (!tool_.isActive() || ticket != tool_.ticket()) return;
        const auto current = host_.read();
        const auto found = current.records.find(record.name);
        if (status != TaskRunner::Status::Success || current.generation != generation
            || found == current.records.end() || found->second.id != record.id || found->second.cloud != record.cloud) {
            result->success = false;
            result->cancelled = status == TaskRunner::Status::Cancelled;
            result->error = status == TaskRunner::Status::Failed ? "预览失败，请重试" : "点云已变化，请重新预览";
        }
        if (result->success) { previewSource_ = record; generation_ = generation; }
        tool_.finishPreview(result, ticket);
    });
}

void ClipController::apply()
{
    if (host_.busy() || !previewSource_ || !tool_.canApply()) return;
    const auto source = *previewSource_;
    const auto result = tool_.result();
    const auto mode = tool_.keepMode();
    // 先撤销临时隐藏，场景快照必须记录实际显示属性而不是预览覆盖状态。
    tool_.stop();
    const auto scene = host_.read();
    const auto found = scene.records.find(source.name);
    if (scene.generation != generation_ || found == scene.records.end()
        || found->second.id != source.id || found->second.cloud != source.cloud
        || !(found->second.appearance == source.appearance)) {
        host_.message("点云已变化，请重新裁切"); return;
    }
    std::vector<CloudScene::Record> outputs;
    auto add = [&](const char* suffix, CloudScene::Cloud::ConstPtr cloud, bool alternate) {
        auto output = source;
        output.id = host_.allocateId();
        output.name = source.name + suffix + std::to_string(output.id);
        while (scene.records.count(output.name)) output.name += "_";
        output.cloud = std::move(cloud);
        if (alternate) output.appearance.color = {255,170,70};
        outputs.push_back(std::move(output));
    };
    if (mode != PolygonClipTool::KeepMode::Outside) add("_inside", result->inside, false);
    if (mode != PolygonClipTool::KeepMode::Inside) add("_outside", result->outside, mode == PolygonClipTool::KeepMode::Both);
    CloudScene::Transaction transaction(scene);
    if (!transaction.replace(source.id, outputs) || !host_.commit(transaction.edit())) {
        host_.message("裁切未应用，请重新预览"); return;
    }
    const auto kept = mode == PolygonClipTool::KeepMode::Inside ? result->inside->size()
        : mode == PolygonClipTool::KeepMode::Outside ? result->outside->size() : result->inside->size() + result->outside->size();
    host_.message(QString("裁切完成: 保留 %1 点").arg(kept));
}
