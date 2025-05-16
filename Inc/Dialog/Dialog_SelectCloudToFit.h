#pragma once
#include "Dialog/headers.h"

class Dialog_SelectCloudToFit : public Dialog_SelectPointCloud {
public:
	Dialog_SelectCloudToFit(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
	~Dialog_SelectCloudToFit();
private slots:
	void confirm();
private:
	void InitializeButtonAndConnect() override;

};