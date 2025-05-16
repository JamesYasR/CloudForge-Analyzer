#include "Loops.h"

App_Proc::App_Proc() {

}
void App_Proc::Slot() {

}
Count_Points::Count_Points(pcl::PointCloud<pcl::PointXYZ>::Ptr Input_cloud, Ui::CloudForgeAnalyzerClass* Input_ui){
	cloud = Input_cloud;
	ui = Input_ui;
	QObject::connect(&timer, &QTimer::timeout,this, &Count_Points::Slot);
	timer.start(200);
}

void Count_Points::Slot(){
	if (cloud->empty()) {
		ui->label_countpoints->setText("points:0");
		return;
	}
	std::string num = std::to_string(cloud->size());
	QString Qnum = QString::fromStdString(num);
	ui->label_countpoints->setText("Points:"+Qnum);
};