#include "Dialog/ParamDialogCurvSeg.h"

ParamDialogCurvSeg::ParamDialogCurvSeg(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "k邻近数","平滑度阈值", "曲率阈值", "最小聚类点数"};
    QVector<QString> defaults = { "20", "1.0", "0.1", "300"};
    setupUI(labels, defaults);
    setWindowTitle("按曲率分割");
}