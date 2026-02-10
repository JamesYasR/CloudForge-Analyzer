#pragma once
#include "Dialog/headers.h"

class FitCloudDialog : public CloudSelectionDialog {
public:
	FitCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
	~FitCloudDialog();
private slots:
	void confirm();
	void cancel();
private:
	void InitializeButtonAndConnect() override;

};