#pragma once
#include "Dialog/headers.h"

class RmCloudDialog : public CloudSelectionDialog {
public:
	RmCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
	~RmCloudDialog();
	std::vector<std::string> Get_toDelete();
private slots:
	void onKeepClicked();
	void onDeleteClicked();
private:
	std::vector<std::string> toDelete;
	void InitializeButtonAndConnect() override;

};
