#include "Dialog/ParamDialogMeausreCy.h"

ParamDialogMeausreCy::ParamDialogMeausreCy(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = {"设计半径（mm）", "容差阈值（mm）", "最多迭代（次）"};
    QVector<QString> defaults = {"1940", "1","2000"};
    setupUI(labels, defaults);
    setWindowTitle("轴线优化");
}
