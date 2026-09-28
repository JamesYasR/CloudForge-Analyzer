#include "Dialog/ParamDialog_Pothole.h"

ParamDialog_Pothole::ParamDialog_Pothole(QWidget* parent)
    : ParamDialogBase(parent)
{
    // 注意: 前 5 项为既有项, 顺序不可变更(流程按下标读取);
    //       局部基准(口径B)相关项一律追加在末尾.
    // "局部基准窗口 W" 默认 90mm: 按缺陷尺寸规范(10~30mm 凹坑)取 W ≈ 3 x 最大坑尺寸,
    // 保证基准窗口内以"周围正常表面"为主(规格 §3.3: W 必须 ≥ 2~3 x 坑尺寸,
    // W=60 时深度误差增大到 -0.08~-0.15mm, W=90 时 -0.02~-0.11mm).
    QVector<QString> labels = {"距离阈值（mm，0=自动）", "聚类容差（mm，0=自动）", "最小点群点数",
                               "形面趋势处理（0=自动/1=不扣除/2=强制扣除）", "点群面积占比上限（%）",
                               "局部基准窗口 W（mm，默认90，建议≥3×最大坑）",
                               "局部阈值 T_local（mm，默认0.35）",
                               "使用局部基准判断（1=是/0=否，默认1）",
                               "热力图显示（0=局部凹陷d/1=到理想柱面e）"};
    QVector<QString> defaults = {"0", "0", "200", "0", "20", "90", "0.35", "1", "0"};
    setupUI(labels, defaults);
    setWindowTitle("凹塘测量参数");
}
