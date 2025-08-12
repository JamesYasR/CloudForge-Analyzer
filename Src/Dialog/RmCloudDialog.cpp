#include "Dialog/RmCloudDialog.h"

RmCloudDialog::RmCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : CloudSelectionDialog(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->exec();
}
RmCloudDialog::~RmCloudDialog() = default;
void RmCloudDialog::onKeepClicked()
{
    toDelete = getUnselectedList();
    this->accept();
}

void RmCloudDialog::onDeleteClicked()
{
    toDelete = getSelectedList();
    this->accept();
}

std::vector<std::string> RmCloudDialog::Get_toDelete() {
    return toDelete;
}

void RmCloudDialog::InitializeButtonAndConnect() {
    QPushButton* keepButton = new QPushButton("保留选中", this);
    QPushButton* deleteButton = new QPushButton("删除选中", this);
    connect(keepButton, &QPushButton::clicked, this, &RmCloudDialog::onKeepClicked);
    connect(deleteButton, &QPushButton::clicked, this, &RmCloudDialog::onDeleteClicked);

    mainLayout->addWidget(keepButton);
    mainLayout->addWidget(deleteButton);
}