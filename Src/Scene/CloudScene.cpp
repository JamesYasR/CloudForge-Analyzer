#include "Scene/CloudScene.h"
#include <algorithm>
#include <set>

bool CloudScene::Transaction::replace(Id source, const std::vector<Record>& outputs)
{
    auto next = edit_.before.records;
    const auto found = std::find_if(next.begin(), next.end(), [source](const auto& item) { return item.second.id == source; });
    if (found == next.end() || outputs.empty()) return false;
    next.erase(found);
    std::set<Id> ids;
    for (const auto& item : edit_.before.records) ids.insert(item.second.id);
    for (const auto& output : outputs) {
        if (!output.id || !output.cloud || output.cloud->empty() || output.name.empty()
            || !ids.insert(output.id).second || !next.emplace(output.name, output).second) return false;
    }
    edit_.after.records.swap(next);
    edit_.description = "多边形裁切";
    return true;
}
