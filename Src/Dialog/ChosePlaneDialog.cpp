#include "Dialog/ChosePlaneDialog.h"

ChosePlaneDialog::ChosePlaneDialog(std::map<std::string, pcl::ModelCoefficients::Ptr> planeMap,
    QWidget* parent)
    : PlaneSelectionDialog(planeMap, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择平面");
    this->exec();
}

ChosePlaneDialog::~ChosePlaneDialog() = default;

void ChosePlaneDialog::InitializeButtonAndConnect() {
    // 先调用父类的按钮初始化
    //PlaneSelectionDialog::InitializeButtonAndConnect();

    // 添加确认和取消按钮
    QHBoxLayout* confirmCancelLayout = new QHBoxLayout;
    QPushButton* confirmBtn = new QPushButton("确认", this);
    QPushButton* cancelBtn = new QPushButton("取消", this);

    connect(confirmBtn, &QPushButton::clicked, this, &ChosePlaneDialog::Confirm);
    connect(cancelBtn, &QPushButton::clicked, this, &ChosePlaneDialog::Cancel);

    confirmCancelLayout->addWidget(confirmBtn);
    confirmCancelLayout->addWidget(cancelBtn);

    // 将确认取消按钮添加到主布局
    mainLayout->addLayout(confirmCancelLayout);
}

void ChosePlaneDialog::Confirm() {
    this->accept();
}

void ChosePlaneDialog::Cancel() {
    this->reject();
}