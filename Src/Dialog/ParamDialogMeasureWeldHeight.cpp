#include "Dialog/ParamDialogMeasureWeldHeight.h"

ParamDialogMeasureWeldHeight::ParamDialogMeasureWeldHeight(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {
        "搜索半径 R (mm)",
        "网格边长 (mm)",
        "RANSAC 阈值 (mm)",
        "最少邻域点数"
    };
    QVector<QString> defaults = {"20.0", "10.0", "0.5", "15"};
    setupUI(labels, defaults);
    setWindowTitle("焊缝高度测量");
}
