#include "Dialog/Dialog_SelectCloudToFit.h"

Dialog_SelectCloudToFit::Dialog_SelectCloudToFit(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : Dialog_SelectPointCloud(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择点云拟合");
    this->exec();
    
}
Dialog_SelectCloudToFit::~Dialog_SelectCloudToFit() = default;

void Dialog_SelectCloudToFit::confirm()
{
    std::vector<std::string> checked_list;
    checked_list = Get_SelectedList();
    if (checked_list.size() != 1) {
        QMessageBox::warning(this, "警告", "只能选择一个点云！");
        ClearAllCheckBoxes();
        return;
    }
    this->accept();
}



void Dialog_SelectCloudToFit::InitializeButtonAndConnect() {
    QPushButton* confirmButton = new QPushButton("确定", this);
    connect(confirmButton, &QPushButton::clicked, this, &Dialog_SelectCloudToFit::confirm);

    mainLayout->addWidget(confirmButton);
}