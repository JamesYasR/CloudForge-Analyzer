#include "Dialog/ParamDialog_ec.h"

ParamDialog_ec::ParamDialog_ec(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "Tolerance", "Minisize", "Maxsize" };
    QVector<QString> defaults = { "0.02", "100", "5000000" };
    setupUI(labels, defaults);
    setWindowTitle("欧式聚类");
}