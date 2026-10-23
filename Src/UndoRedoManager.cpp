#include "UndoRedoManager.h"
#include <set>

void UndoRedoManager::pushState(const std::string& description, CloudScene::Snapshot scene, bool preserveCamera)
{
    undoStack_.push_back({description, std::move(scene), preserveCamera});
    redoStack_.clear();
    while (undoStack_.size() > MAX_HISTORY) undoStack_.pop_front();
}

std::optional<UndoRedoManager::Entry> UndoRedoManager::undo(CloudScene::Snapshot current)
{
    if (undoStack_.empty()) return std::nullopt;
    redoStack_.push_back({undoStack_.back().description, std::move(current), undoStack_.back().preserveCamera});
    auto result = std::move(undoStack_.back());
    undoStack_.pop_back();
    return result;
}

std::optional<UndoRedoManager::Entry> UndoRedoManager::redo(CloudScene::Snapshot current)
{
    if (redoStack_.empty()) return std::nullopt;
    undoStack_.push_back({redoStack_.back().description, std::move(current), redoStack_.back().preserveCamera});
    auto result = std::move(redoStack_.back());
    redoStack_.pop_back();
    return result;
}

void UndoRedoManager::clear() { undoStack_.clear(); redoStack_.clear(); }

std::size_t UndoRedoManager::retainedBytes() const
{
    std::set<const CloudScene::Cloud*> counted;
    std::size_t bytes = 0;
    for (const auto* stack : {&undoStack_, &redoStack_}) for (const auto& entry : *stack)
        for (const auto& item : entry.scene.records) if (item.second.cloud && counted.insert(item.second.cloud.get()).second)
            bytes += item.second.cloud->points.capacity() * sizeof(pcl::PointXYZ);
    return bytes;
}
