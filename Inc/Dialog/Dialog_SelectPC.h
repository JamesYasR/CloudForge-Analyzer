#pragma once
#include "Dialog/headers.h"

class Dialog_SelectPC : public QDialog {
    Q_OBJECT

public:
    explicit Dialog_SelectPC(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm,QWidget* parent = nullptr);
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
