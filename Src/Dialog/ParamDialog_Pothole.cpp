#include "Dialog/ParamDialog_Pothole.h"
#include <QFormLayout>
#include <QFrame>
#include <QToolButton>
#include <QHBoxLayout>
#include <QDoubleValidator>
#include <QIntValidator>

ParamDialog_Pothole::ParamDialog_Pothole(QWidget* parent):ParamDialogBase(parent)
{
    setObjectName("potholeParameters");setWindowTitle("凹坑测量");setMinimumWidth(350);
    const QVector<QString> defaults={"0","0","200","0","20","90","0.35","1","0"};
    for (int i=0;i<defaults.size();++i) {
        auto edit=new QLineEdit(defaults[i],this);edit->setObjectName(QString("potholeParam%1").arg(i));paramEdits.append(edit);
        if (i!=1 && i!=2 && i!=5 && i!=6) edit->hide();
    }
    for (int i:{1,5,6}) paramEdits[i]->setValidator(new QDoubleValidator(0,1e6,6,paramEdits[i]));
    paramEdits[2]->setValidator(new QIntValidator(1,10000000,paramEdits[2]));
    auto form=new QFormLayout();form->addRow("凹陷阈值 (mm)",paramEdits[6]);form->addRow("局部基准窗口 (mm)",paramEdits[5]);mainLayout->addLayout(form);
    auto toggle=new QToolButton(this);toggle->setObjectName("potholeAdvanced");toggle->setText("更多设置");toggle->setCheckable(true);toggle->setArrowType(Qt::RightArrow);toggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);mainLayout->addWidget(toggle);
    auto advanced=new QFrame(this);advanced->setObjectName("potholeAdvancedFields");auto extra=new QFormLayout(advanced);extra->setContentsMargins(0,4,0,4);
    extra->addRow("聚类容差 (mm，0为自动)",paramEdits[1]);extra->addRow("最小点数",paramEdits[2]);mainLayout->addWidget(advanced);advanced->hide();
    connect(toggle,&QToolButton::toggled,this,[this,toggle,advanced](bool checked) { advanced->setVisible(checked);toggle->setArrowType(checked?Qt::DownArrow:Qt::RightArrow);adjustSize(); });
    auto buttons=new QHBoxLayout();auto cancel=new QPushButton("取消",this);auto start=new QPushButton("开始测量",this);start->setDefault(true);
    buttons->addStretch();buttons->addWidget(cancel);buttons->addWidget(start);mainLayout->addLayout(buttons);
    connect(cancel,&QPushButton::clicked,this,&QDialog::reject);connect(start,&QPushButton::clicked,this,&ParamDialog_Pothole::accept);
    setStyleSheet("QPushButton { border: 1px solid palette(mid); border-radius: 0; padding: 6px 12px; }");
}
