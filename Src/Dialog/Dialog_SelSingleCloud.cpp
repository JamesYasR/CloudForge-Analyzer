#include "Dialog/Dialog_SelSingleCloud.h"

Dialog_SelSingleCloud::Dialog_SelSingleCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : Dialog_SelectPC(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->exec();
    this->setWindowTitle("选择点云拟合");
}
Dialog_SelSingleCloud::~Dialog_SelSingleCloud() = default;

void Dialog_SelSingleCloud::confirm()
{
    std::vector<std::string> checked_list;
    checked_list = Get_SelectedList();
    if (checked_list.size() != 1) {
        QMessageBox::warning(this, "警告", "只能选择一个点云！");
        ClearAllCheckBoxes();
    }
    Sleceted = checked_list[0];
    this->accept();
}

std::string Dialog_SelSingleCloud::Get_Selected(){
    return Sleceted;
}

void Dialog_SelSingleCloud::InitializeButtonAndConnect() {
    QPushButton* confirmButton = new QPushButton("确定", this);
    connect(confirmButton, &QPushButton::clicked, this, &Dialog_SelSingleCloud::confirm);

    mainLayout->addWidget(confirmButton);
}