#include "Dialog/ChoseLineDialog.h"
#include "Basic/Basic.h"

ChoseLineDialog::ChoseLineDialog(std::map<std::string, Line> Linem, QWidget* parent)
    : LineSelectionDialog(Linem, parent)
{
    InitializeButtonAndConnect();
    this->setWindowTitle("选择直线");
    this->exec();
}

ChoseLineDialog::~ChoseLineDialog() = default;

void ChoseLineDialog::InitializeButtonAndConnect() {
    QPushButton* Button1 = new QPushButton("确定", this);
    QPushButton* Button2 = new QPushButton("取消", this);
    connect(Button1, &QPushButton::clicked, this, &ChoseLineDialog::Confirm);
    connect(Button2, &QPushButton::clicked, this, &ChoseLineDialog::Cancel);
    mainLayout->addWidget(Button1);
    mainLayout->addWidget(Button2);
}

void ChoseLineDialog::Confirm()
{
    this->accept();
}

void ChoseLineDialog::Cancel() {
    this->reject();
}