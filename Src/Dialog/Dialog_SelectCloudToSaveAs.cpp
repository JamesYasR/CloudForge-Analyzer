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
    QPushButton* Button1 = new QPushButton("另存为", this);
    QPushButton* Button2 = new QPushButton("取消", this);
    connect(Button1, &QPushButton::clicked, this, &Dialog_SelectCloudToSaveAs::Confirm);
    connect(Button2, &QPushButton::clicked, this, &Dialog_SelectCloudToSaveAs::Cancel);
    mainLayout->addWidget(Button1);
    mainLayout->addWidget(Button2);
}


bool Dialog_SelectCloudToSaveAs::SaveSelectedClouds(const QString& filePath, QString& infoMsg) {
    auto SaveList = Get_SelectedList();
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Merged(new pcl::PointCloud<pcl::PointXYZ>);
    Cloud_Merged->clear();
    for (const auto& key : SaveList) {
        if (cloudmap.find(key) != cloudmap.end()) {
            *Cloud_Merged += *(cloudmap[key]);
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

void Dialog_SelectCloudToSaveAs::Confirm()
{
    this->accept();
}

void Dialog_SelectCloudToSaveAs::Cancel(){
    this->reject();
}