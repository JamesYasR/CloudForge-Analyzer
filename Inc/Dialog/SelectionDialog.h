#pragma once
#include "Dialog/headers.h"
#include "Basic/Basic.h"

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

class LineSelectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit LineSelectionDialog(
        const std::map<std::string, Line>& lineMap,  // 修改为const引用
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
    std::map<std::string, Line> lineMap;  // 统一命名为lineMap
};


class PlaneSelectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit PlaneSelectionDialog(
        const std::map<std::string, pcl::ModelCoefficients::Ptr>& planeMap,
        QWidget* parent = nullptr);

    std::vector<std::string> getSelectedList() const;
    std::vector<std::string> getUnselectedList() const;
    void clearAllCheckBoxes();
    void selectAllCheckBoxes();

protected:
    virtual void InitializeButtonAndConnect();
    void InitializeScrollArea();

    QVBoxLayout* mainLayout;
    QList<QCheckBox*> checkBoxes;
    QStringList planeIds;
    std::map<std::string, pcl::ModelCoefficients::Ptr> planeMap;
};