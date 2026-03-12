#include "Dialog/ParamDialogMeaArc.h"

ParamDialogMeaArc::ParamDialogMeaArc(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "切片厚度因数","积分容差","样条曲线降采样点数","端口虚拟点外推比例"};
    QVector<QString> defaults = { "5.0","1e-5","30","0.03"};
    setupUI(labels, defaults);
    setWindowTitle("样条曲线测量");
}