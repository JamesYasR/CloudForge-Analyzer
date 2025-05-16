#include "Dialog/ParamDialog_vg.h"

ParamDialog_vg::ParamDialog_vg(QWidget* parent) : QDialog(parent)
{
    // 创建布局和控件
    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* label = new QLabel("leafsize", this);
    lineEdit = new QLineEdit(this);//做一个输入框
    QPushButton* okButton = new QPushButton("确定", this);
    lineEdit->setText(QString::number(0.2));

    layout->addWidget(label);
    layout->addWidget(lineEdit);
    layout->addWidget(okButton);

    // 设置对话框的标题和大小
    setWindowTitle("体素滤波");
    resize(300, 100);

    // 连接信号和槽
    connect(okButton, &QPushButton::clicked, this, &ParamDialog_vg::accept);
}

QString ParamDialog_vg::getParam() const
{
    return lineEdit->text();
}

void ParamDialog_vg::accept()
{
    if (lineEdit->text().isEmpty())
    {
        QMessageBox::warning(this, "警告", "请输入参数！");
        return;
    }
    QDialog::accept(); // 调用父类的accept方法，关闭对话框
}