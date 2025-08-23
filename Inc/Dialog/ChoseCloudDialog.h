#pragma once
#include "Dialog/headers.h"

class ChoseCloudDialog : public CloudSelectionDialog {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit ChoseCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
    ~ChoseCloudDialog();
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};