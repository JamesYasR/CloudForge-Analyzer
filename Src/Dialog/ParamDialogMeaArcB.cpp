#include "Dialog/ParamDialogMeaArcB.h"

ParamDialogMeaArcB::ParamDialogMeaArcB(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "切片厚度因数","积分容差","B样条次数(阶数-1)","B样条控制点数量","平滑因子" };
    QVector<QString> defaults = { "5.0","1e-5","3","15","0.01" };
    setupUI(labels, defaults);
    setWindowTitle("B样条曲线拟合测量");
}