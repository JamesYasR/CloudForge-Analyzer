#pragma once
#include "Dialog/headers.h"

class ParamDialog_FittingCylinder : public QDialog
{
    Q_OBJECT

public:
    explicit ParamDialog_FittingCylinder(QWidget* parent = nullptr);

    QString* getParam() const; // 获取输入的参数

private slots:
    void accept() override; // 重写accept方法

private:
    QLineEdit* lineEdit1;
    QLineEdit* lineEdit2;
    QLineEdit* lineEdit3;
};