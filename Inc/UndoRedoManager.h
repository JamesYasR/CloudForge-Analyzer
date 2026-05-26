#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <string>
#include <deque>
#include "Basic/Basic.h"

// 基于 CloudMap/ColorMap 快照的撤销/重做管理器。
// 所有 cloud 以 shared_ptr 持有，快照仅复制 map 结构(引用计数+1)，
// 不复制点云数据，因此内存开销 = 被删除但在历史中存活的旧点云。
class UndoRedoManager {
public:
    static constexpr int MAX_HISTORY = 20; // 可调，控制内存上限

    // 推送当前状态到撤销栈（应在 CloudMap 被修改前调用）
    void pushState(const std::string& description,
        const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
        const std::map<std::string, ColorManager>& colorMap);

    bool canUndo() const;
    bool canRedo() const;

    // 返回本次操作的描述文本，便于 UI 展示
    std::string undo(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
        std::map<std::string, ColorManager>& colorMap);

    std::string redo(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudMap,
        std::map<std::string, ColorManager>& colorMap);

    void clear();

private:
    struct Entry {
        std::string description;
        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudMap;
        std::map<std::string, ColorManager> colorMap;
    };

    std::deque<Entry> undoStack_;
    std::deque<Entry> redoStack_;
};
