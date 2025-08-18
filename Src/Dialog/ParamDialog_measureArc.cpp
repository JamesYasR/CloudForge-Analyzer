#include "Dialog/ParamDialog_measureArc.h"

ParamDialog_measureArc::ParamDialog_measureArc(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"KSearch", "DistanceThreshold", "MaxIterations"};
    QVector<QString> defaults = {"9", "2.1325", "500"};
    setupUI(labels, defaults);
    setWindowTitle("圆柱弧测量");
}
