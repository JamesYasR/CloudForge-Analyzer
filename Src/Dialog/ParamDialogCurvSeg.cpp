#include "Dialog/ParamDialogCurvSeg.h"

ParamDialogCurvSeg::ParamDialogCurvSeg(QWidget* parent)
    : ParamDialogBase(parent)
{
    QVector<QString> labels = { "curvature_threshold", "normal_radius", "min_cluster_size" };
    QVector<QString> defaults = { "0.02", "0.03", "100" };
    setupUI(labels, defaults);
    setWindowTitle("按曲率分割");
}