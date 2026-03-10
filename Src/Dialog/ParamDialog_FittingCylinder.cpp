#include "Dialog/ParamDialog_FittingCylinder.h"

ParamDialog_FittingCylinder::ParamDialog_FittingCylinder(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"KSearch", "误差容忍（mm）", "最多迭代（次）","半径猜测（mm）"};
    QVector<QString> defaults = {"60", "3", "1000","1940.00"};
    setupUI(labels, defaults);
    setWindowTitle("RANSAC圆柱拟合");
}
