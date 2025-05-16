#include "Dialog/ParamDialog_ec.h"

ParamDialog_ec::ParamDialog_ec(QWidget* parent) : QDialog(parent)
{
    // 创建布局和控件
    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* label1 = new QLabel("Tolerance", this);
    lineEdit1 = new QLineEdit(this);//做一个输入框
    QLabel* label2 = new QLabel("Minisize", this);
    lineEdit2 = new QLineEdit(this);//做一个输入框
    QLabel* label3 = new QLabel("Maxsize", this);
    lineEdit3 = new QLineEdit(this);//做一个输入框
    QPushButton* okButton = new QPushButton("确定", this);

    lineEdit1->setText(QString::number(0.02));
    lineEdit2->setText(QString::number(100));
    lineEdit3->setText(QString::number(5000000));

    layout->addWidget(label1);
    layout->addWidget(lineEdit1);
    layout->addWidget(label2);
    layout->addWidget(lineEdit2);
    layout->addWidget(label3);
    layout->addWidget(lineEdit3);
    layout->addWidget(okButton);

    // 设置对话框的标题和大小
    setWindowTitle("欧式聚类");
    resize(300, 100);

    // 连接信号和槽
    connect(okButton, &QPushButton::clicked, this, &ParamDialog_ec::accept);
}

QString* ParamDialog_ec::getParam() const
{
    QString paras[3];
    paras[0] = lineEdit1->text();
    paras[1] = lineEdit2->text();
    paras[2] = lineEdit3->text();
    return paras;
}

void ParamDialog_ec::accept()
{
    if (lineEdit1->text().isEmpty() || lineEdit2->text().isEmpty() || lineEdit3->text().isEmpty())
    {
        QMessageBox::warning(this, "警告", "请输入参数！");
        return;
    }
    QDialog::accept(); // 调用父类的accept方法，关闭对话框
}
