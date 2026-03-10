#pragma once
#include "Dialog/headers.h"
#include "Dialog/CloudSelectionDialog.h"
#include <iostream>
class ChoseCloudDialog : public CloudSelectionDialog {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit ChoseCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, std::string title = "选择点云", QWidget* parent = nullptr);
    ~ChoseCloudDialog();
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};