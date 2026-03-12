#pragma once
#include "Dialog/headers.h"
#include "Dialog/CySelectionDialog.h"
#include "Basic/Basic.h"

class ChoseCyDialog : public CySelectionDialog {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit ChoseCyDialog(std::map<std::string, pcl::ModelCoefficients::Ptr> Cym,QWidget* parent = nullptr);
    ~ChoseCyDialog();
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};