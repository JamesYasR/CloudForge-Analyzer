#pragma once
#include "Dialog/headers.h"
#include "Dialog/SelectionDialog.h"
#include "Basic/Basic.h"

class ChosePlaneDialog : public PlaneSelectionDialog {
    Q_OBJECT

public:
    explicit ChosePlaneDialog(std::map<std::string, pcl::ModelCoefficients::Ptr> planeMap,
        QWidget* parent = nullptr);
    ~ChosePlaneDialog();

private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};