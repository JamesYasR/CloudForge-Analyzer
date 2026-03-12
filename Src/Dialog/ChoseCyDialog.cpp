#include "Dialog/ChoseCyDialog.h"
#include "Basic/Basic.h"

ChoseCyDialog::ChoseCyDialog(std::map<std::string, pcl::ModelCoefficients::Ptr> Cym, QWidget* parent)
    : CySelectionDialog(Cym, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择圆柱");
    this->exec();
}

ChoseCyDialog::~ChoseCyDialog() = default;

void ChoseCyDialog::InitializeButtonAndConnect() {
    QPushButton* Button1 = new QPushButton("确定", this);
    QPushButton* Button2 = new QPushButton("取消", this);
    connect(Button1, &QPushButton::clicked, this, &ChoseCyDialog::Confirm);
    connect(Button2, &QPushButton::clicked, this, &ChoseCyDialog::Cancel);
    mainLayout->addWidget(Button1);
    mainLayout->addWidget(Button2);
}

void ChoseCyDialog::Confirm()
{
    this->accept();
}

void ChoseCyDialog::Cancel() {
    this->reject();
}