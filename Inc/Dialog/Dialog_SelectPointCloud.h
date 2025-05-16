#pragma once
#include "Dialog/headers.h"

class Dialog_SelectPointCloud : public QDialog {
    Q_OBJECT

public:
    explicit Dialog_SelectPointCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm,QWidget* parent = nullptr);
    void InitializeScrollAreaOfPointCloud();
    virtual void InitializeButtonAndConnect();
    void ClearAllCheckBoxes();
    void SelectAllCheckBoxes();
    QVBoxLayout* mainLayout;
    QList<QCheckBox*> m_checkBoxes;
    QStringList m_cloudIds;

    std::vector<std::string> Get_SelectedList();
    std::vector<std::string> Get_unSelectedList();

    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudmap;
    std::map<std::string, ColorManager> colormap;

private slots:
    
private:
    
};
//创建此类的意义:实现选择点云的核心功能，即获取程序内存中点云的名称，颜色，点数列表以供选择。子类可能需要不同的交互按钮或是其他控件，如删除/保留点云的功能
//就需要两个不同的按钮进行交互，他已经具有获取 选择，未选择列表的功能