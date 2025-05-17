#pragma once
#include "Dialog/headers.h"

class Dialog_SelectCloudToSaveAs : public Dialog_SelectPointCloud {
    Q_OBJECT

public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit Dialog_SelectCloudToSaveAs(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
    ~Dialog_SelectCloudToSaveAs();
    bool SaveSelectedClouds(const QString& filePath, QString& infoMsg);
private slots:
    void Confirm();
    void Cancel();

private:
    void InitializeButtonAndConnect() override;
};