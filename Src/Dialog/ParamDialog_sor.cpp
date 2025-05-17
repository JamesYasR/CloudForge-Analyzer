#include "Dialog/ParamDialog_sor.h"

ParamDialog_sor::ParamDialog_sor(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"mean_k", "std_dev_mul_thresh"};
    QVector<QString> defaults = {"20", "0.2"};
    setupUI(labels, defaults);
    setWindowTitle("统计离群滤波器");
}