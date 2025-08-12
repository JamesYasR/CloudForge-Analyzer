#include "Dialog/FitCloudDialog.h"

FitCloudDialog::FitCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : CloudSelectionDialog(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择点云拟合");
    this->exec();
    
}
FitCloudDialog::~FitCloudDialog() = default;

void FitCloudDialog::confirm()
{
    std::vector<std::string> checked_list;
    checked_list = getSelectedList();
    if (checked_list.size() != 1) {
        QMessageBox::warning(this, "警告", "只能选择一个点云！");
        clearAllCheckBoxes();
        return;
    }
    this->accept();
}



void FitCloudDialog::InitializeButtonAndConnect() {
    QPushButton* confirmButton = new QPushButton("确定", this);
    connect(confirmButton, &QPushButton::clicked, this, &FitCloudDialog::confirm);

    mainLayout->addWidget(confirmButton);
}