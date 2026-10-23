#include "CloudForgeAnalyzer.h"
#include "Basic/PolygonClipTool.h"
#include "Basic/MeasurementPresentation.h"
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>

// 过渡外观：旧 maps 仍是存储，控制器只能读取记录或提交事务。
// 身份索引不另存数据/颜色/显示属性，避免形成两套可写场景。
CloudScene::Snapshot CloudForgeAnalyzer::captureCloudScene()
{
    Q_ASSERT(QThread::currentThread() == thread());
    CloudScene::Snapshot scene;
    scene.generation = m_sceneGeneration;
    const auto actors = viewer->getCloudActorMap();
    for (const auto& item : CloudMap) {
        if (!item.second) continue;
        auto& identity = m_cloudIdentities[item.first];
        if (!identity.first || identity.second != item.second) identity = {m_nextCloudId++, item.second};
        CloudScene::Record record;
        record.id = identity.first;
        record.name = item.first;
        record.cloud = item.second;
        const auto color = ColorMap.find(item.first);
        if (color != ColorMap.end()) record.appearance.color = {color->second.r, color->second.g, color->second.b};
        const auto found = actors->find(item.first);
        if (found != actors->end()) {
            auto actor = found->second.actor;
            auto& view = record.appearance;
            for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
                view.transform[row*4+col] = actor->GetMatrix()->GetElement(row,col);
            view.pointSize = actor->GetProperty()->GetPointSize();
            view.opacity = actor->GetProperty()->GetOpacity();
            view.visible = actor->GetVisibility() != 0;
            // 预览和热图的临时隐藏不属于业务可见性，不能写进历史。
            if (m_presentation) if (const auto visible = m_presentation->sourceVisibility(actor)) view.visible = *visible;
            if (m_clipTool) if (const auto visible = m_clipTool->sourceVisibility(actor)) view.visible = *visible;
            view.pickable = actor->GetPickable() != 0;
        }
        scene.records.emplace(item.first, std::move(record));
    }
    return scene;
}

void CloudForgeAnalyzer::applyCloudAppearance(const std::string& name, const CloudScene::Appearance& view)
{
    const auto actors = viewer->getCloudActorMap();
    const auto found = actors->find(name);
    if (found == actors->end()) return;
    auto actor = found->second.actor;
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
        matrix->SetElement(row,col,view.transform[row*4+col]);
    actor->SetUserMatrix(matrix);
    actor->GetProperty()->SetPointSize(view.pointSize);
    actor->GetProperty()->SetOpacity(view.opacity);
    actor->SetVisibility(view.visible);
    actor->SetPickable(view.pickable);
}

CloudForgeAnalyzer::PreparedCloudScene CloudForgeAnalyzer::prepareCloudScene(const CloudScene::Snapshot& scene)
{
    // 全部容器在交换前构造完；没有逐件删除/添加形成的半个场景。
    PreparedCloudScene prepared;
    prepared.scene = scene;
    for (const auto& item : scene.records) {
        const auto& record = item.second;
        prepared.clouds.emplace(item.first, std::const_pointer_cast<CloudScene::Cloud>(record.cloud));
        const auto& rgb = record.appearance.color;
        prepared.colors.emplace(item.first, ColorManager(rgb[0],rgb[1],rgb[2]));
        prepared.identities.emplace(item.first, std::make_pair(record.id, record.cloud));
    }
    return prepared;
}

void CloudForgeAnalyzer::installCloudScene(PreparedCloudScene prepared, bool preserveCamera)
{
    CloudMap.swap(prepared.clouds);
    ColorMap.swap(prepared.colors);
    m_cloudIdentities.swap(prepared.identities);
    ++m_sceneGeneration;
    rebuildCloudVisualization(preserveCamera);
    for (const auto& item : prepared.scene.records) applyCloudAppearance(item.first, item.second.appearance);
    ui->winOfAnalyzer->renderWindow()->Render();
    ui->winOfAnalyzer->update();
}

bool CloudForgeAnalyzer::commitCloudEdit(const CloudScene::Edit& edit)
{
    Q_ASSERT(QThread::currentThread() == thread());
    const auto current = captureCloudScene();
    if (current.generation != edit.before.generation || current.records.size() != edit.before.records.size()) return false;
    for (const auto& item : edit.before.records) {
        const auto found = current.records.find(item.first);
        if (found == current.records.end() || found->second.id != item.second.id
            || found->second.cloud != item.second.cloud || !(found->second.appearance == item.second.appearance)) return false;
    }
    // 源对象更新只提交一次；预览/失败/取消不会进入此处。
    auto prepared = prepareCloudScene(edit.after);
    m_undoRedoManager.pushState(edit.description, current, true);
    installCloudScene(std::move(prepared), true);
    return true;
}
