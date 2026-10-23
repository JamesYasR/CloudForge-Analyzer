#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include <utility>

// GUI 线程持有场景；计算只接收 ConstPtr。这里不保存 actor 或控件。
namespace CloudScene {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Id = std::uint64_t;

struct Appearance {
    std::array<double, 16> transform{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    std::array<double, 3> color{255,255,255};
    double pointSize = 1;
    double opacity = 1;
    bool visible = true;
    bool pickable = true;
    bool operator==(const Appearance& other) const {
        return transform == other.transform && color == other.color
            && pointSize == other.pointSize && opacity == other.opacity
            && visible == other.visible && pickable == other.pickable;
    }
};

struct Record {
    Id id = 0;
    std::string name;
    Cloud::ConstPtr cloud;
    Appearance appearance;
};

struct Snapshot {
    // 撤销只恢复内容，世代绝不倒退，防止旧任务在 undo 后重新获得提交资格。
    std::uint64_t generation = 0;
    std::map<std::string, Record> records;
};

struct Edit {
    Snapshot before;
    Snapshot after;
    std::string description;
};

// 事务只准备新值，不改场景。未提交即丢弃，多个输出共享一个历史条目。
class Transaction {
public:
    explicit Transaction(Snapshot before) : edit_{before, std::move(before), {}} {}
    bool replace(Id source, const std::vector<Record>& outputs);
    const Edit& edit() const { return edit_; }
private:
    Edit edit_;
};
}
