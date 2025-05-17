#include "Dialog/ParamDialog_FittingCylinder.h"

ParamDialog_FittingCylinder::ParamDialog_FittingCylinder(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"KSearch", "DistanceThreshold", "MaxIterations"};
    QVector<QString> defaults = {"20", "0.02", "500"};
    setupUI(labels, defaults);
    setWindowTitle("RANSAC圆柱拟合");
}
