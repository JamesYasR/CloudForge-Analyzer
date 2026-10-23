#pragma once
#include "Scene/CloudScene.h"
#include <deque>
#include <optional>

// 快照共享点云数据，属性为纯值；不保存 VTK 对象。
class UndoRedoManager {
public:
    static constexpr int MAX_HISTORY = 20;
    struct Entry {
        std::string description;
        CloudScene::Snapshot scene;
        bool preserveCamera = false;
    };
    void pushState(const std::string& description, CloudScene::Snapshot scene, bool preserveCamera = false);
    bool canUndo() const { return !undoStack_.empty(); }
    bool canRedo() const { return !redoStack_.empty(); }
    const Entry* nextUndo() const { return canUndo() ? &undoStack_.back() : nullptr; }
    const Entry* nextRedo() const { return canRedo() ? &redoStack_.back() : nullptr; }
    std::size_t retainedBytes() const;
    std::optional<Entry> undo(CloudScene::Snapshot current);
    std::optional<Entry> redo(CloudScene::Snapshot current);
    void clear();
private:
    std::deque<Entry> undoStack_, redoStack_;
};
