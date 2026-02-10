#include "Dialog/ChoseCloudDialog.h"

ChoseCloudDialog::ChoseCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : CloudSelectionDialog(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择点云");
    //this->exec();
}

ChoseCloudDialog::~ChoseCloudDialog() = default;

void ChoseCloudDialog::InitializeButtonAndConnect() {
    QPushButton* Button1 = new QPushButton("确定", this);
    QPushButton* Button2 = new QPushButton("取消", this);
    connect(Button1, &QPushButton::clicked, this, &ChoseCloudDialog::Confirm);
    connect(Button2, &QPushButton::clicked, this, &ChoseCloudDialog::Cancel);
    mainLayout->addWidget(Button1);
    mainLayout->addWidget(Button2);
}

void ChoseCloudDialog::Confirm()
{
    this->accept();
}

void ChoseCloudDialog::Cancel() {
    this->reject();
}