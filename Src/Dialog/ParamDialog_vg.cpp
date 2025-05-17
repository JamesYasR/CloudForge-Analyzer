#include "Dialog/ParamDialog_vg.h"

ParamDialog_vg::ParamDialog_vg(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"leafsize"};
    QVector<QString> defaults = {"0.2"};
    setupUI(labels, defaults);
    setWindowTitle("体素滤波");
}