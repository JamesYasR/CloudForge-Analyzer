#include "Dialog/Dialog_SelectCloudToSaveAs.h"

Dialog_SelectCloudToSaveAs::Dialog_SelectCloudToSaveAs(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : Dialog_SelectPointCloud(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择点云另存为");
    this->exec();
}

Dialog_SelectCloudToSaveAs::~Dialog_SelectCloudToSaveAs() = default;

void Dialog_SelectCloudToSaveAs::InitializeButtonAndConnect() {
    QPushButton* Button = new QPushButton("另存为", this);
    connect(Button, &QPushButton::clicked, this, &Dialog_SelectCloudToSaveAs::Confirm);
    mainLayout->addWidget(Button);
}

void Dialog_SelectCloudToSaveAs::Confirm()
{
    this->accept();
}
