#include "UndoRedoManager.h"

void UndoRedoManager::pushState(
    const std::string& description,
    const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
    const std::map<std::string, ColorManager>& colorMap)
{
    // 新操作意味着重做栈失效
    redoStack_.clear();

    Entry entry;
    entry.description = description;
    entry.cloudMap = cloudMap;
    entry.colorMap = colorMap;

    undoStack_.push_back(std::move(entry));

    // 超过上限则淘汰最旧记录
    while (static_cast<int>(undoStack_.size()) > MAX_HISTORY) {
        undoStack_.pop_front();
    }
}

bool UndoRedoManager::canUndo() const {
    return !undoStack_.empty();
}

bool UndoRedoManager::canRedo() const {
    return !redoStack_.empty();
}

std::string UndoRedoManager::undo(
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
    std::map<std::string, ColorManager>& colorMap)
{
    if (undoStack_.empty()) return {};

    // 保存当前状态到重做栈
    Entry redoEntry;
    redoEntry.description = undoStack_.back().description;
    redoEntry.cloudMap = cloudMap;
    redoEntry.colorMap = colorMap;
    redoStack_.push_back(std::move(redoEntry));

    // 从撤销栈恢复
    Entry entry = std::move(undoStack_.back());
    undoStack_.pop_back();

    cloudMap = entry.cloudMap;
    colorMap = entry.colorMap;
    return entry.description;
}

std::string UndoRedoManager::redo(
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
    std::map<std::string, ColorManager>& colorMap)
{
    if (redoStack_.empty()) return {};

    // 保存当前状态到撤销栈
    Entry undoEntry;
    undoEntry.description = redoStack_.back().description;
    undoEntry.cloudMap = cloudMap;
    undoEntry.colorMap = colorMap;
    undoStack_.push_back(std::move(undoEntry));

    // 从重做栈恢复
    Entry entry = std::move(redoStack_.back());
    redoStack_.pop_back();

    cloudMap = entry.cloudMap;
    colorMap = entry.colorMap;
    return entry.description;
}

void UndoRedoManager::clear() {
    undoStack_.clear();
    redoStack_.clear();
}
