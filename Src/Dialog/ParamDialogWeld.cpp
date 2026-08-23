#include "Dialog/ParamDialogWeld.h"

ParamDialogWeld::ParamDialogWeld(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"阈值系数k（越小检测越多）", "聚类容差（mm）", "最小簇点数", "约束权重λ"};
    QVector<QString> defaults = {"3.0", "2.0", "20", "1.0"};
    setupUI(labels, defaults);
    setWindowTitle("焊缝检测参数");
}
