#pragma once
#include "Dialog/headers.h"

class Dialog_SelectCloudToSaveAs : public Dialog_SelectPointCloud {
    Q_OBJECT

public:
public:
    using PCLVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

    explicit Dialog_SelectCloudToSaveAs(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
    ~Dialog_SelectCloudToSaveAs();
private slots:
    void Saveas();

private:
    void InitializeButtonAndConnect() override;
};