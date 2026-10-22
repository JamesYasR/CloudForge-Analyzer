#include "Dialog/ParamDialog_FittingPlane.h"

ParamDialog_FittingPlane::ParamDialog_FittingPlane(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "平面拟合距离阈值(mm)", "最大迭代次数" };
    QVector<QString> defaults = { "1.0", "1000" };
    setupUI(labels, defaults);
    setWindowTitle("平面拟合参数");
}