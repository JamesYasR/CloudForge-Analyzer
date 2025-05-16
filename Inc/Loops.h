#pragma once
#include <QtWidgets/QApplication>
#include <QTimer>
#include "CloudForgeAnalyzer.h"

class App_Proc : public QObject {
	Q_OBJECT
public:
	App_Proc();
	virtual void Slot();
	QTimer timer;
};

class Count_Points : public App_Proc{
	Q_OBJECT
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Ui::CloudForgeAnalyzerClass* ui;
	CloudForgeAnalyzer* win;
	Count_Points(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud, Ui::CloudForgeAnalyzerClass* Input_ui);
	void Slot() override;
};