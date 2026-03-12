#pragma once
#include "Dialog/headers.h"
#include "Basic/Basic.h"

class CySelectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit CySelectionDialog(
        const std::map<std::string, pcl::ModelCoefficients::Ptr>& cylinderMap,  // 修改为const引用
        QWidget* parent = nullptr);

    std::vector<std::string> getSelectedList() const;
    std::vector<std::string> getUnselectedList() const;
    void clearAllCheckBoxes();
    void selectAllCheckBoxes();

protected:
    void InitializeScrollArea();
    virtual void InitializeButtonAndConnect();

    QVBoxLayout* mainLayout;
    QList<QCheckBox*> checkBoxes;
    QStringList cloudIds;
    std::map<std::string, pcl::ModelCoefficients::Ptr> CylinderMap;
};