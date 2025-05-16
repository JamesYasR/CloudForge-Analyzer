#include "Dialog/ParamDialog_sor.h"

ParamDialog_sor::ParamDialog_sor(QWidget* parent) : QDialog(parent)
{
    // 创建布局和控件
    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* label1 = new QLabel("mean_k", this);
    lineEdit1 = new QLineEdit(this);//做一个输入框
    QLabel* label2 = new QLabel("std_dev_mul_thresh", this);
    lineEdit2 = new QLineEdit(this);//做一个输入框
    QPushButton* okButton = new QPushButton("确定", this);
    lineEdit1->setText(QString::number(20));
    lineEdit2->setText(QString::number(0.2));

    layout->addWidget(label1);
    layout->addWidget(lineEdit1);
    layout->addWidget(label2);
    layout->addWidget(lineEdit2);
    layout->addWidget(okButton);

    // 设置对话框的标题和大小
    setWindowTitle("统计离群滤波器");
    resize(300, 100);

    // 连接信号和槽
    connect(okButton, &QPushButton::clicked, this, &ParamDialog_sor::accept);
}

QString* ParamDialog_sor::getParam() const
{
    QString paras[2];
    paras[0] = lineEdit1->text();
    paras[1] = lineEdit2->text();
    return paras;
}

void ParamDialog_sor::accept()
{
    if (lineEdit1->text().isEmpty() || lineEdit1->text().isEmpty())
    {
        QMessageBox::warning(this, "警告", "请输入参数！");
        return;
    }
    QDialog::accept(); // 调用父类的accept方法，关闭对话框
}