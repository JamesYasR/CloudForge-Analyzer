#include "Dialog/ParamDialog_FittingLine.h"

ParamDialog_FittingLine::ParamDialog_FittingLine(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "距离阈值", "最大迭代次数" };
    QVector<QString> defaults = { "0.01", "1000" };
    setupUI(labels, defaults);
    setWindowTitle("RANSAC直线拟合");
}