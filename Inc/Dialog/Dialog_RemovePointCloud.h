#pragma once
#include "Dialog/headers.h"

class Dialog_RemovePointCloud : public Dialog_SelectPointCloud {
public:
	Dialog_RemovePointCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
	~Dialog_RemovePointCloud();
	std::vector<std::string> Get_toDelete();
private slots:
	void onKeepClicked();
	void onDeleteClicked();
private:
	void InitializeButtonAndConnect() override;
	std::vector<std::string> toDelete;

};
