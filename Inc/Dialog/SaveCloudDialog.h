#pragma once
#include "Dialog/headers.h"

class SaveCloudDialog : public CloudSelectionDialog {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit SaveCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
    ~SaveCloudDialog();
    bool SaveSelectedClouds(const QString& filePath, QString& infoMsg);
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};