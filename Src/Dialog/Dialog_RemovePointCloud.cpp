#include "Dialog/Dialog_RemovePointCloud.h"

Dialog_RemovePointCloud::Dialog_RemovePointCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : Dialog_SelectPointCloud(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->exec();
}
Dialog_RemovePointCloud::~Dialog_RemovePointCloud() = default;
void Dialog_RemovePointCloud::onKeepClicked()
{
    toDelete = Get_unSelectedList();
    this->accept();
}

void Dialog_RemovePointCloud::onDeleteClicked()
{
    toDelete = Get_SelectedList();
    this->accept();
}

std::vector<std::string> Dialog_RemovePointCloud::Get_toDelete() {
    return toDelete;
}

void Dialog_RemovePointCloud::InitializeButtonAndConnect() {
    QPushButton* keepButton = new QPushButton("保留选中", this);
    QPushButton* deleteButton = new QPushButton("删除选中", this);
    connect(keepButton, &QPushButton::clicked, this, &Dialog_RemovePointCloud::onKeepClicked);
    connect(deleteButton, &QPushButton::clicked, this, &Dialog_RemovePointCloud::onDeleteClicked);

    mainLayout->addWidget(keepButton);
    mainLayout->addWidget(deleteButton);
}