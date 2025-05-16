#pragma once
#include "Dialog/headers.h"

class Dialog_saveas : public QDialog {
    Q_OBJECT

public:
public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit Dialog_saveas(PCLVisualizerPtr viewer, QWidget* parent = nullptr);

private slots:
    void saveas();

private:
    PCLVisualizerPtr m_viewer;
    QList<QCheckBox*> m_checkBoxes;
    QStringList m_cloudIds; // PCL使用字符串作为云ID
};