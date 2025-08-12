#include "Dialog/ParamDialogBase.h"
#include "Dialog/headers.h"
ParamDialogBase::ParamDialogBase(QWidget* parent)
    : QDialog(parent), mainLayout(new QVBoxLayout(this))
{
    setLayout(mainLayout);
}

void ParamDialogBase::setupUI(const QVector<QString>& labels, const QVector<QString>& defaults)
{
    paramEdits.clear();
    for (int i = 0; i < labels.size(); ++i) {
        QLabel* label = new QLabel(labels[i], this);
        QLineEdit* edit = new QLineEdit(this);
        if (i < defaults.size())
            edit->setText(defaults[i]);
        paramEdits.append(edit);
        mainLayout->addWidget(label);
        mainLayout->addWidget(edit);
    }
    QPushButton* okButton = new QPushButton("确定", this);
    mainLayout->addWidget(okButton);
    connect(okButton, &QPushButton::clicked, this, &ParamDialogBase::accept);
    //setFixedSize(300, 100 + 40 * labels.size());
}

QVector<QString> ParamDialogBase::getParams() const
{
    QVector<QString> params;
    for (auto* edit : paramEdits)
        params.append(edit->text());
    return params;
}

void ParamDialogBase::accept()
{
    for (auto* edit : paramEdits) {
        if (edit->text().isEmpty()) {
            QMessageBox::warning(this, "warning", "pls input");
            return;
        }
    }
   // emit paramsConfirmed(getParams()); // 新增：发射信号
    QDialog::accept();
}