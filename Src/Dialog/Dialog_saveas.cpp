#include "Dialog/Dialog_saveas.h"

Dialog_saveas::Dialog_saveas(PCLVisualizerPtr viewer, QWidget* parent)
    : QDialog(parent), m_viewer(viewer)
{
    // 主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 获取当前所有点云ID
    auto cloudActorMap = m_viewer->getCloudActorMap();
    for (const auto& entry : *cloudActorMap) {
        m_cloudIds << QString::fromStdString(entry.first);
    }

    // 滚动区域
    QScrollArea* scrollArea = new QScrollArea;
    QWidget* contentWidget = new QWidget;
    QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);

    // 为每个点云添加复选框
    for (const QString& id : m_cloudIds) {
        QHBoxLayout* rowLayout = new QHBoxLayout;

        // 颜色标签（示例颜色，实际可从CloudActor获取）
        QLabel* colorLabel = new QLabel;
        colorLabel->setFixedSize(20, 20);
        colorLabel->setStyleSheet("background-color: #888888;");

        // 复选框
        QCheckBox* checkBox = new QCheckBox(id);

        rowLayout->addWidget(colorLabel);
        rowLayout->addWidget(checkBox);
        contentLayout->addLayout(rowLayout);
        m_checkBoxes.append(checkBox);
    }

    scrollArea->setWidget(contentWidget);

    // 保存按钮
    QPushButton* saveButton = new QPushButton("保存");
    connect(saveButton, &QPushButton::clicked, this, &Dialog_saveas::saveas);

    mainLayout->addWidget(scrollArea);
    mainLayout->addWidget(saveButton);
    setLayout(mainLayout);
    resize(400, 300);
    this->exec();
}

// 保存功能实现
void Dialog_saveas::saveas()
{
    // 1. 获取选中的点云ID
    QStringList selectedIds;
    for (int i = 0; i < m_checkBoxes.size(); ++i) {
        if (m_checkBoxes[i]->isChecked()) {
            selectedIds << m_cloudIds[i];
        }
    }

    if (selectedIds.isEmpty()) {
        QMessageBox::warning(this, "警告", "请至少选择一个点云！");
        return;
    }

    // 2. 初始化合并点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto cloudActorMap = m_viewer->getCloudActorMap();

    // 3. 遍历处理每个选中的点云
    for (const QString& id : selectedIds) {
        auto it = cloudActorMap->find(id.toStdString());
        if (it != cloudActorMap->end()) {
            const auto& cloudActor = it->second;

            // 4. 安全获取PolyData（关键修正）
            vtkSmartPointer<vtkPolyData> polydata;
#if VTK_MAJOR_VERSION >= 9
            vtkDataSet* dataset = cloudActor.actor->GetMapper()->GetInput();
            polydata = vtkPolyData::SafeDownCast(dataset);
#else
            vtkDataObject* dataObj = cloudActor.actor->GetMapper()->GetInput();
            polydata = vtkPolyData::SafeDownCast(dataObj);
#endif

            // 5. 验证数据有效性
            if (!polydata || !polydata->GetPoints()) {
                qWarning() << "无效的点云数据：" << id;
                continue;
            }

            // 6. 提取点数据
            vtkPoints* points = polydata->GetPoints();
            const vtkIdType numPoints = points->GetNumberOfPoints();

            // 7. 转换为PCL格式
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.width = numPoints;
            cloud.height = 1;
            cloud.resize(numPoints);

            // 8. 填充点坐标
            for (vtkIdType i = 0; i < numPoints; ++i) {
                double pos[3];
                points->GetPoint(i, pos);
                cloud.points[i].x = static_cast<float>(pos[0]);
                cloud.points[i].y = static_cast<float>(pos[1]);
                cloud.points[i].z = static_cast<float>(pos[2]);
            }

            // 9. 获取变换矩阵（修正行列顺序）
            vtkSmartPointer<vtkMatrix4x4> matrix = cloudActor.actor->GetMatrix();
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    transform(i, j) = matrix->GetElement(j, i); // 修正点云变换
                }
            }

            // 10. 应用变换
            pcl::transformPointCloud(cloud, cloud, transform);

            // 11. 合并点云
            *mergedCloud += cloud;
        }
    }

    // 12. 检查空点云
    if (mergedCloud->empty()) {
        QMessageBox::critical(this, "错误", "合并后的点云为空！");
        return;
    }

    // 13. 文件保存对话框
    QDir saveDir(QDir::current().filePath("PCDfiles"));
    if (!saveDir.exists()) saveDir.mkpath(".");

    QString defaultName = QString("cloud_%1.pcd")
        .arg(QDateTime::currentDateTime().toString("yyyyMMddHHmmss"));

    QString filePath = QFileDialog::getSaveFileName(
        this,
        "保存点云文件",
        saveDir.filePath(defaultName),
        "PCD文件 (*.pcd)"
    );

    if (!filePath.isEmpty()) {
        // 14. 强制添加扩展名
        if (QFileInfo(filePath).suffix().compare("pcd", Qt::CaseInsensitive) != 0) {
            filePath += ".pcd";
        }

        // 15. 执行保存
        try {
            if (pcl::io::savePCDFileBinaryCompressed(filePath.toStdString(), *mergedCloud) == -1) {
                throw std::runtime_error("未知错误");
            }
            QMessageBox::information(this, "成功",
                QString("成功保存 %1 个点云到：\n%2").arg(selectedIds.size()).arg(filePath));
            accept();
        }
        catch (const std::exception& e) {
            QMessageBox::critical(this, "错误",
                QString("保存失败：\n%1").arg(e.what()));
        }
    }
}