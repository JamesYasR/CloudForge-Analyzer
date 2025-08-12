#pragma once
#include "Dialog/headers.h"

class CloudSelectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit CloudSelectionDialog(
        const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudMap,
        const std::map<std::string, ColorManager> colorMap,
        QWidget* parent = nullptr);

    std::vector<std::string> getSelectedList();
    std::vector<std::string> getUnselectedList();
    void clearAllCheckBoxes();
    void selectAllCheckBoxes();


protected:
    void InitializeScrollArea();
    virtual void InitializeButtonAndConnect();

    QVBoxLayout* mainLayout;
    QList<QCheckBox*> checkBoxes;
    QStringList cloudIds;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudMap;
    std::map<std::string, ColorManager> colorMap;
};