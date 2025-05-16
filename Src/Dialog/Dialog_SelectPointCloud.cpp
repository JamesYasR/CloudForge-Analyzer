#include "Dialog/Dialog_SelectPointCloud.h"

Dialog_SelectPointCloud::Dialog_SelectPointCloud(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudm, std::map<std::string, ColorManager> colorm, QWidget* parent)
    : QDialog(parent)
{
    cloudmap = cloudm;
    colormap = colorm;
    mainLayout = new QVBoxLayout(this);


    InitializeScrollAreaOfPointCloud();
    InitializeButtonAndConnect();

    setLayout(mainLayout);
    resize(400, 300);

}




void Dialog_SelectPointCloud::InitializeScrollAreaOfPointCloud() {
    std::vector<std::string> cloudIds;
    for (const auto& entry : cloudmap) {
        cloudIds.push_back(entry.first);
    }

    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    for (const auto& cloudId : cloudIds) {
        QHBoxLayout* rowLayout = new QHBoxLayout;
        double r = 0.8, g = 0.8, b = 0.8;
        auto it = colormap.find(cloudId);
        qDebug() << cloudId;
        if (it == colormap.end()) {
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
        BoxLabel->setText(QString::fromStdString("Points:" + std::to_string(cloudmap[cloudId]->size())));

        rowLayout->addWidget(checkBox);
        rowLayout->addWidget(BoxLabel);
        contentLayout->addLayout(rowLayout);

        m_cloudIds.append(QString::fromStdString(cloudId)); // 确保 m_cloudIds 和 m_checkBoxes 的大小一致
        m_checkBoxes.append(checkBox);
    }
    scrollArea->setWidget(contentWidget);
    mainLayout->addWidget(scrollArea);
}

void Dialog_SelectPointCloud::InitializeButtonAndConnect() {

}
std::vector<std::string> Dialog_SelectPointCloud::Get_SelectedList() {
    std::vector<std::string> SelectList;
    for (int i = 0; i < m_checkBoxes.size(); ++i) {
        if (m_checkBoxes[i]->isChecked()) {
            SelectList.push_back(m_cloudIds[i].toStdString());
        }
    }
    return SelectList;
}
std::vector<std::string> Dialog_SelectPointCloud::Get_unSelectedList() {
    std::vector<std::string> unSelectList;
    for (int i = 0; i < m_checkBoxes.size(); ++i) {
        if (!m_checkBoxes[i]->isChecked()) {
            unSelectList.push_back(m_cloudIds[i].toStdString());
        }
    }
    return unSelectList;
}

void Dialog_SelectPointCloud::ClearAllCheckBoxes() {
    for (QCheckBox* checkBox : m_checkBoxes) {
        if (checkBox) {
            checkBox->setChecked(false); // 将每个复选框的选中状态设置为false
        }
    }
}

void Dialog_SelectPointCloud::SelectAllCheckBoxes() {
    for (QCheckBox* checkBox : m_checkBoxes) {
        if (checkBox) {
            checkBox->setChecked(true); // 将每个复选框的选中状态设置为false
        }
    }
}