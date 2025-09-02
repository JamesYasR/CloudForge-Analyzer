#pragma once
#include "Dialog/headers.h"
#include "Dialog/LineSelectionDialog.h"
#include "Basic/Basic.h"

class ChoseLineDialog : public LineSelectionDialog {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit ChoseLineDialog(std::map<std::string, Line> Linem,QWidget* parent = nullptr);
    ~ChoseLineDialog();
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};