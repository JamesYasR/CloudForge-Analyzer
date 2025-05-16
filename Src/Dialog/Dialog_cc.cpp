#include "Dialog/Dialog_cc.h"

Dialog_cc::Dialog_cc(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : Dialog_SelectPC(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->exec();
}
Dialog_cc::~Dialog_cc() = default;
void Dialog_cc::onKeepClicked()
{
    toDelete = Get_unSelectedList();
    this->accept();
}

void Dialog_cc::onDeleteClicked()
{
    toDelete = Get_SelectedList();
    this->accept();
}

std::vector<std::string> Dialog_cc::Get_toDelete() {
    return toDelete;
}

void Dialog_cc::InitializeButtonAndConnect() {
    QPushButton* keepButton = new QPushButton("保留选中", this);
    QPushButton* deleteButton = new QPushButton("删除选中", this);
    connect(keepButton, &QPushButton::clicked, this, &Dialog_cc::onKeepClicked);
    connect(deleteButton, &QPushButton::clicked, this, &Dialog_cc::onDeleteClicked);

    mainLayout->addWidget(keepButton);
    mainLayout->addWidget(deleteButton);
}