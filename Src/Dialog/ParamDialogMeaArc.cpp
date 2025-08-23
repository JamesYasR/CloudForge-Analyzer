#include "Dialog/ParamDialogMeaArc.h"

ParamDialogMeaArc::ParamDialogMeaArc(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "base_radius" };
    QVector<QString> defaults = { "0.5" };
    setupUI(labels, defaults);
    setWindowTitle("²âµØÏßËÑË÷");
}