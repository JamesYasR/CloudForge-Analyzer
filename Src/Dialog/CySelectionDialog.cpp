#include "Dialog/CySelectionDialog.h"

CySelectionDialog::CySelectionDialog(
    const std::map<std::string, pcl::ModelCoefficients::Ptr>& cylinderMap,  // 修改为const引用
    QWidget* parent)
    : QDialog(parent),
    CylinderMap(cylinderMap)  // 使用成员初始化列表
{
    mainLayout = new QVBoxLayout(this);

    InitializeScrollArea();
    InitializeButtonAndConnect();  // 取消注释

    resize(400, 300);
}

void CySelectionDialog::InitializeScrollArea() {
    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    for (const auto& [id, line] : CylinderMap) {  // 直接遍历lineMap
        QHBoxLayout* rowLayout = new QHBoxLayout;

        QColor color(255, 255, 255);

        QLabel* colorLabel = new QLabel;
        colorLabel->setFixedSize(20, 20);
        colorLabel->setStyleSheet(QString("background-color: %1;").arg(color.name()));
        rowLayout->addWidget(colorLabel);

        QCheckBox* checkBox = new QCheckBox(QString::fromStdString(id));

        rowLayout->addWidget(checkBox);
        rowLayout->addStretch();

        contentLayout->addLayout(rowLayout);
        cloudIds.append(QString::fromStdString(id));
        checkBoxes.append(checkBox);
    }

    scrollArea->setWidget(contentWidget);
    scrollArea->setWidgetResizable(true);
    mainLayout->addWidget(scrollArea);
}

void CySelectionDialog::InitializeButtonAndConnect() {
    QHBoxLayout* buttonLayout = new QHBoxLayout;

    QPushButton* selectAllBtn = new QPushButton("Select All");
    QPushButton* clearAllBtn = new QPushButton("Clear All");
    //QPushButton* confirmBtn = new QPushButton("Confirm");

    connect(selectAllBtn, &QPushButton::clicked, this, &CySelectionDialog::selectAllCheckBoxes);
    connect(clearAllBtn, &QPushButton::clicked, this, &CySelectionDialog::clearAllCheckBoxes);
    //connect(confirmBtn, &QPushButton::clicked, this, &QDialog::accept);

    buttonLayout->addWidget(selectAllBtn);
    buttonLayout->addWidget(clearAllBtn);
   // buttonLayout->addWidget(confirmBtn);

    mainLayout->addLayout(buttonLayout);
}

std::vector<std::string> CySelectionDialog::getSelectedList() const {
    std::vector<std::string> selected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && checkBoxes[i]->isChecked()) {
            selected.push_back(cloudIds[i].toStdString());
        }
    }
    return selected;
}

std::vector<std::string> CySelectionDialog::getUnselectedList() const {
    std::vector<std::string> unselected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && !checkBoxes[i]->isChecked()) {
            unselected.push_back(cloudIds[i].toStdString());
        }
    }
    return unselected;
}

void CySelectionDialog::clearAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(false);
    }
}

void CySelectionDialog::selectAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(true);
    }
}