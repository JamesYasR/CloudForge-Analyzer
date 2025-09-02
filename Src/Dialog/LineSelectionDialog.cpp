#include "Dialog/LineSelectionDialog.h"

LineSelectionDialog::LineSelectionDialog(
    const std::map<std::string, Line>& lineMap,  // 修改为const引用
    QWidget* parent)
    : QDialog(parent),
    lineMap(lineMap)  // 使用成员初始化列表
{
    mainLayout = new QVBoxLayout(this);

    InitializeScrollArea();
    InitializeButtonAndConnect();  // 取消注释

    resize(400, 300);
}

void LineSelectionDialog::InitializeScrollArea() {
    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    for (const auto& [id, line] : lineMap) {  // 直接遍历lineMap
        QHBoxLayout* rowLayout = new QHBoxLayout;

        // 使用线对象的颜色
        QColor color(line.color.r, line.color.g, line.color.b);

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

void LineSelectionDialog::InitializeButtonAndConnect() {
    QHBoxLayout* buttonLayout = new QHBoxLayout;

    QPushButton* selectAllBtn = new QPushButton("Select All");
    QPushButton* clearAllBtn = new QPushButton("Clear All");
    //QPushButton* confirmBtn = new QPushButton("Confirm");

    connect(selectAllBtn, &QPushButton::clicked, this, &LineSelectionDialog::selectAllCheckBoxes);
    connect(clearAllBtn, &QPushButton::clicked, this, &LineSelectionDialog::clearAllCheckBoxes);
    //connect(confirmBtn, &QPushButton::clicked, this, &QDialog::accept);

    buttonLayout->addWidget(selectAllBtn);
    buttonLayout->addWidget(clearAllBtn);
   // buttonLayout->addWidget(confirmBtn);

    mainLayout->addLayout(buttonLayout);
}

std::vector<std::string> LineSelectionDialog::getSelectedList() const {
    std::vector<std::string> selected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && checkBoxes[i]->isChecked()) {
            selected.push_back(cloudIds[i].toStdString());
        }
    }
    return selected;
}

std::vector<std::string> LineSelectionDialog::getUnselectedList() const {
    std::vector<std::string> unselected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && !checkBoxes[i]->isChecked()) {
            unselected.push_back(cloudIds[i].toStdString());
        }
    }
    return unselected;
}

void LineSelectionDialog::clearAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(false);
    }
}

void LineSelectionDialog::selectAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(true);
    }
}