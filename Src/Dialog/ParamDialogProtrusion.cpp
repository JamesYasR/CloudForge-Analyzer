#include "Dialog/​​ParamDialogProtrusion.h"

ParamDialogProtrusion::ParamDialogProtrusion(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {
        "高度阈值 (m)",
        "搜索半径 (m)",
        "最小聚类点数"
    };

    QVector<QString> defaults = {
        "0.14",  // 典型焊缝高度1-5mm
        "14.0",   // 30mm搜索半径
        "16"      // 最小聚类点数
    };

    setupUI(labels, defaults);
    setWindowTitle("焊缝突起点检测");
}