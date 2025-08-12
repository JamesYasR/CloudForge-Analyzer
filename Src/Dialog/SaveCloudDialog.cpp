#include "Dialog/SaveCloudDialog.h"

SaveCloudDialog::SaveCloudDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : CloudSelectionDialog(cloudm, colorm, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择点云另存为");
    this->exec();
}

SaveCloudDialog::~SaveCloudDialog() = default;

void SaveCloudDialog::InitializeButtonAndConnect() {
    QPushButton* Button1 = new QPushButton("另存为", this);
    QPushButton* Button2 = new QPushButton("取消", this);
    connect(Button1, &QPushButton::clicked, this, &SaveCloudDialog::Confirm);
    connect(Button2, &QPushButton::clicked, this, &SaveCloudDialog::Cancel);
    mainLayout->addWidget(Button1);
    mainLayout->addWidget(Button2);
}


bool SaveCloudDialog::SaveSelectedClouds(const QString& filePath, QString& infoMsg) {
    auto SaveList = getSelectedList();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Merged(new pcl::PointCloud<pcl::PointXYZ>);
    Cloud_Merged->clear();
    for (const auto& key : SaveList) {
        if (cloudMap.find(key) != cloudMap.end()) {
            *Cloud_Merged += *(cloudMap[key]);
        }
        else {
            infoMsg = QString("点云 %1 不存在").arg(QString::fromStdString(key));
            return false;
        }
    }
    if (!filePath.isEmpty()) {
        std::string savePath = filePath.toStdString();
        if (QFileInfo(filePath).suffix().compare("pcd", Qt::CaseInsensitive) != 0) {
            savePath += ".pcd";
        }
        if (pcl::io::savePCDFileBinaryCompressed(savePath, *Cloud_Merged) == -1) {
            infoMsg = "保存失败";
            return false;
        }
        else {
            infoMsg = QString("成功保存 %1 个点云到：\n%2").arg(SaveList.size()).arg(QString::fromStdString(savePath));
            return true;
        }
    }
    infoMsg = "文件路径为空";
    return false;
}

void SaveCloudDialog::Confirm()
{
    this->accept();
}

void SaveCloudDialog::Cancel(){
    this->reject();
}