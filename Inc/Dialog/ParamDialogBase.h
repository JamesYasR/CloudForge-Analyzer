#pragma once
#include <QDialog>
#include <QVector>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

class ParamDialogBase: public QDialog {
    Q_OBJECT
public:
    explicit ParamDialogBase(QWidget* parent = nullptr);
    virtual ~ParamDialogBase() = default;

    // 设置参数标签和默认值
    void setupUI(const QVector<QString>& labels, const QVector<QString>& defaults);

    // 获取所有参数
    QVector<QString> getParams() const;
//signals:
    //void paramsConfirmed(const QVector<QString>& params); // 新增：参数确认信号

protected slots:
    virtual void accept() override;

protected:
    QVector<QLineEdit*> paramEdits;
    QVBoxLayout* mainLayout;
};