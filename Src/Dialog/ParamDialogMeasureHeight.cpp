#include "Dialog/ParamDialogMeasureHeight.h"

ParamDialogMeasureHeight::ParamDialogMeasureHeight(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels  = { "邻近点数量", "迭代次数" };
    QVector<QString> defaults = { "10", "5" };
    setupUI(labels, defaults);
    setWindowTitle("高度测量参数");
}
