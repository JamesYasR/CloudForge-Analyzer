#pragma once
#include "Dialog/headers.h"

class Dialog_SelSingleCloud : public Dialog_SelectPC {
public:
	Dialog_SelSingleCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent = nullptr);
	~Dialog_SelSingleCloud();
	std::string Get_Selected();
private slots:
	void confirm();
private:
	void InitializeButtonAndConnect() override;
	std::string Sleceted;

};