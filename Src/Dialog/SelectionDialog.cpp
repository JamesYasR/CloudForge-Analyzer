#include "Dialog/SelectionDialog.h"

CloudSelectionDialog::CloudSelectionDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudmap, std::map<std::string, ColorManager> colormap, QWidget* parent)
    : QDialog(parent)
{
    cloudMap = cloudmap;
    colorMap = colormap;
    mainLayout = new QVBoxLayout(this);


    InitializeScrollArea();
    InitializeButtonAndConnect();

    setLayout(mainLayout);
    resize(400, 300);

}




void CloudSelectionDialog::InitializeScrollArea() {
    std::vector<std::string> temp_cloudIds;
    for (const auto& entry : cloudMap) {
        temp_cloudIds.push_back(entry.first);
    }

    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    for (const auto& cloudId : temp_cloudIds) {
        QHBoxLayout* rowLayout = new QHBoxLayout;
        double r = 0.8, g = 0.8, b = 0.8;
        auto it = colorMap.find(cloudId);
        qDebug() << cloudId;
        if (it == colorMap.end()) {
            break;
        }
        ColorManager cm = it->second;
        r = cm.r;
        g = cm.g;
        b = cm.b;
        QColor color(r, g, b);

        QLabel* colorLabel = new QLabel;
        colorLabel->setFixedSize(20, 20);
        colorLabel->setStyleSheet(QString("background-color: %1;").arg(color.name()));
        colorLabel->update();
        rowLayout->addWidget(colorLabel);

        QCheckBox* checkBox = new QCheckBox(QString::fromStdString(cloudId));
        QLabel* BoxLabel = new QLabel;
        BoxLabel->setText(QString::fromStdString("Points:" + std::to_string(cloudMap[cloudId]->size())));

        rowLayout->addWidget(checkBox);
        rowLayout->addWidget(BoxLabel);
        contentLayout->addLayout(rowLayout);

        cloudIds.append(QString::fromStdString(cloudId)); // 确保 m_cloudIds 和 m_checkBoxes 的大小一致
        checkBoxes.append(checkBox);
    }
    scrollArea->setWidget(contentWidget);
    mainLayout->addWidget(scrollArea);
}

void CloudSelectionDialog::InitializeButtonAndConnect() {
    QHBoxLayout* buttonLayout = new QHBoxLayout;

    QPushButton* selectAllBtn = new QPushButton("Select All");
    QPushButton* clearAllBtn = new QPushButton("Clear All");
    //QPushButton* confirmBtn = new QPushButton("Confirm");

    connect(selectAllBtn, &QPushButton::clicked, this, &CloudSelectionDialog::selectAllCheckBoxes);
    connect(clearAllBtn, &QPushButton::clicked, this, &CloudSelectionDialog::clearAllCheckBoxes);
    //connect(confirmBtn, &QPushButton::clicked, this, &QDialog::accept);

    buttonLayout->addWidget(selectAllBtn);
    buttonLayout->addWidget(clearAllBtn);
    // buttonLayout->addWidget(confirmBtn);
    mainLayout->addLayout(buttonLayout);
 }

std::vector<std::string> CloudSelectionDialog::getSelectedList() {
    std::vector<std::string> SelectList;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i]->isChecked()) {
            SelectList.push_back(cloudIds[i].toStdString());
        }
    }
    return SelectList;
}
std::vector<std::string> CloudSelectionDialog::getUnselectedList() {
    std::vector<std::string> unSelectList;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (!checkBoxes[i]->isChecked()) {
            unSelectList.push_back(cloudIds[i].toStdString());
        }
    }
    return unSelectList;
}

void CloudSelectionDialog::clearAllCheckBoxes() {
    for (QCheckBox* checkBox : checkBoxes) {
        if (checkBox) {
            checkBox->setChecked(false); // 将每个复选框的选中状态设置为false
        }
    }
}

void CloudSelectionDialog::selectAllCheckBoxes() {
    for (QCheckBox* checkBox : checkBoxes) {
        if (checkBox) {
            checkBox->setChecked(true); // 将每个复选框的选中状态设置为false
        }
    }
}






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




PlaneSelectionDialog::PlaneSelectionDialog(
    const std::map<std::string, pcl::ModelCoefficients::Ptr>& planeMap,
    QWidget* parent)
    : QDialog(parent),
    planeMap(planeMap)
{
    mainLayout = new QVBoxLayout(this);

    InitializeScrollArea();
    InitializeButtonAndConnect();

    setWindowTitle("选择平面");
    resize(400, 300);
}

void PlaneSelectionDialog::InitializeScrollArea() {
    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    for (const auto& [id, planeCoeff] : planeMap) {
        QHBoxLayout* rowLayout = new QHBoxLayout;

        // 平面颜色标签
        QLabel* colorLabel = new QLabel;
        colorLabel->setFixedSize(20, 20);
        colorLabel->setStyleSheet("background-color: #4CAF50; border: 1px solid #ccc;");
        rowLayout->addWidget(colorLabel);

        // 平面名称复选框
        QCheckBox* checkBox = new QCheckBox(QString::fromStdString(id));
        rowLayout->addWidget(checkBox);
        rowLayout->addStretch();

        // 显示平面方程信息
        if (planeCoeff && planeCoeff->values.size() >= 4) {
            QLabel* infoLabel = new QLabel(QString("A:%1 B:%2 C:%3 D:%4")
                .arg(planeCoeff->values[0], 0, 'f', 2)
                .arg(planeCoeff->values[1], 0, 'f', 2)
                .arg(planeCoeff->values[2], 0, 'f', 2)
                .arg(planeCoeff->values[3], 0, 'f', 2));
            infoLabel->setStyleSheet("color: #666; font-size: 9pt;");
            rowLayout->addWidget(infoLabel);
        }

        contentLayout->addLayout(rowLayout);
        planeIds.append(QString::fromStdString(id));
        checkBoxes.append(checkBox);
    }

    scrollArea->setWidget(contentWidget);
    scrollArea->setWidgetResizable(true);
    mainLayout->addWidget(scrollArea);
}

void PlaneSelectionDialog::InitializeButtonAndConnect() {
    QHBoxLayout* buttonLayout = new QHBoxLayout;

    QPushButton* selectAllBtn = new QPushButton("全选");
    QPushButton* clearAllBtn = new QPushButton("清空");

    connect(selectAllBtn, &QPushButton::clicked, this, &PlaneSelectionDialog::selectAllCheckBoxes);
    connect(clearAllBtn, &QPushButton::clicked, this, &PlaneSelectionDialog::clearAllCheckBoxes);

    buttonLayout->addWidget(selectAllBtn);
    buttonLayout->addWidget(clearAllBtn);

    mainLayout->addLayout(buttonLayout);
}

std::vector<std::string> PlaneSelectionDialog::getSelectedList() const {
    std::vector<std::string> selected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && checkBoxes[i]->isChecked()) {
            selected.push_back(planeIds[i].toStdString());
        }
    }
    return selected;
}

std::vector<std::string> PlaneSelectionDialog::getUnselectedList() const {
    std::vector<std::string> unselected;
    for (int i = 0; i < checkBoxes.size(); ++i) {
        if (checkBoxes[i] && !checkBoxes[i]->isChecked()) {
            unselected.push_back(planeIds[i].toStdString());
        }
    }
    return unselected;
}

void PlaneSelectionDialog::clearAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(false);
    }
}

void PlaneSelectionDialog::selectAllCheckBoxes() {
    for (auto* checkBox : checkBoxes) {
        if (checkBox) checkBox->setChecked(true);
    }
}