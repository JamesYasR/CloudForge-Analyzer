#include "Dialog/CloudSelectionDialog.h"

CloudSelectionDialog::CloudSelectionDialog(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudmap, std::map<std::string, ColorManager> colormap, QWidget* parent)
    : QDialog(parent)
{
    cloudMap = cloudmap;
    colorMap = colormap;
    mainLayout = new QVBoxLayout(this);


    InitializeScrollArea();
    //InitializeButtonAndConnect();

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