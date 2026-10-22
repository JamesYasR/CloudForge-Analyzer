#include "Basic/ArcMeasurementTool.h"
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleValidator>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QGroupBox>
#include <QVBoxLayout>
#include <cmath>

ArcMeasurementTool::ArcMeasurementTool(QWidget* view):view_(view)
{
    dock_=new QDockWidget("圆柱弧长",view->window());dock_->setObjectName("arcMeasurementDock");dock_->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
    auto body=new QWidget(dock_);body->setMinimumWidth(310);auto layout=new QVBoxLayout(body);form_=new QFormLayout();
    cloud_=new QComboBox(body);cloud_->setObjectName("arcCloud");cloud_->setPlaceholderText("请选择点云");
    reference_=new QComboBox(body);reference_->setObjectName("arcReference");reference_->setPlaceholderText("请选择圆柱");
    method_=new QComboBox(body);method_->setObjectName("arcMethod");method_->addItems({"Cardinal 样条（插值）","B 样条（平滑）"});method_->setCurrentIndex(1);
    position_=new QComboBox(body);position_->setObjectName("arcPosition");position_->addItems({"中线","选取位置"});
    form_->addRow("测量点云",cloud_);form_->addRow("参考圆柱",reference_);form_->addRow("拟合方式",method_);form_->addRow("测量位置",position_);
    const QStringList defaults={"5.0","1e-5","30","0.03","3","15","0.01","0.5"};
    for (int i=0;i<defaults.size();++i) { auto field=new QLineEdit(defaults[i],body);field->setObjectName(QString("arcParameter%1").arg(i));field->setValidator(new QDoubleValidator(field));fields_.push_back(field); }
    form_->addRow("邻域半径 (mm)",fields_[7]);layout->addLayout(form_);
    advanced_=new QGroupBox("测量设置",body);advanced_->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Maximum);auto settings=new QVBoxLayout(advanced_);auto common=new QFormLayout();common->addRow("切片厚度因数",fields_[0]);common->addRow("积分容差",fields_[1]);settings->addLayout(common);
    methodFields_=new QStackedWidget(advanced_);
    auto cardinal=new QWidget(methodFields_);auto cform=new QFormLayout(cardinal);cform->setContentsMargins(0,0,0,0);cform->addRow("降采样点数",fields_[2]);cform->addRow("端点外推比例",fields_[3]);
    auto bspline=new QWidget(methodFields_);auto bform=new QFormLayout(bspline);bform->setContentsMargins(0,0,0,0);bform->addRow("B 样条次数",fields_[4]);bform->addRow("控制点数量",fields_[5]);bform->addRow("平滑因子",fields_[6]);
    methodFields_->addWidget(cardinal);methodFields_->addWidget(bspline);methodFields_->setCurrentIndex(1);settings->addWidget(methodFields_);layout->addWidget(advanced_);
    auto actions=new QHBoxLayout();measure_=new QPushButton("开始测量",body);measure_->setObjectName("arcMeasure");measure_->setToolTip("开始测量（Ctrl+Enter）");cancel_=new QPushButton("取消计算",body);cancel_->setObjectName("arcCancel");actions->addWidget(measure_);actions->addWidget(cancel_);layout->addLayout(actions);
    clear_=new QPushButton("清除结果",body);clear_->setObjectName("arcClear");auto close=new QPushButton("关闭面板",body);close->setObjectName("arcClose");layout->addWidget(clear_);layout->addWidget(close);layout->addStretch();
    body->setStyleSheet("QPushButton { border: 1px solid palette(mid); border-radius: 0; padding: 6px 8px; } QPushButton:hover { background: palette(light); } QPushButton:disabled { color: palette(mid); }");dock_->setWidget(body);
    if (auto main=qobject_cast<QMainWindow*>(view->window())) { main->addDockWidget(Qt::LeftDockWidgetArea,dock_);main->resizeDocks({dock_},{350},Qt::Horizontal); }
    dock_->hide();dock_->installEventFilter(this);qApp->installEventFilter(this);
    connect(measure_,&QPushButton::clicked,this,&ArcMeasurementTool::measureRequested);
    connect(cancel_,&QPushButton::clicked,this,[this]() { cancelling_=true;updateUi();emit cancelRequested(); });
    connect(clear_,&QPushButton::clicked,this,&ArcMeasurementTool::clearRequested);connect(close,&QPushButton::clicked,this,&ArcMeasurementTool::stop);
    connect(method_,&QComboBox::currentIndexChanged,methodFields_,&QStackedWidget::setCurrentIndex);
    for (auto combo:{cloud_,reference_}) connect(combo,&QComboBox::currentIndexChanged,this,[this]() { updateUi(); });updateUi();
}
ArcMeasurementTool::~ArcMeasurementTool() { stop();qApp->removeEventFilter(this);delete dock_; }
bool ArcMeasurementTool::start(bool geodesic,const std::vector<std::string>& clouds,const std::vector<std::string>& cylinders)
{
    stop();if (clouds.empty()) return false;
    geodesic_=geodesic;
    { QSignalBlocker a(cloud_),b(reference_);cloud_->clear();reference_->clear();for (const auto& name:clouds) cloud_->addItem(QString::fromStdString(name));for (const auto& name:cylinders) reference_->addItem(QString::fromStdString(name));
      cloud_->setCurrentIndex(clouds.size()==1 ? 0:-1);reference_->setCurrentIndex(cylinders.size()==1 ? 0:-1); }
    for (auto field:{static_cast<QWidget*>(reference_),static_cast<QWidget*>(method_),static_cast<QWidget*>(position_)}) { field->setVisible(!geodesic_);form_->labelForField(field)->setVisible(!geodesic_); }
    fields_[7]->setVisible(geodesic_);form_->labelForField(fields_[7])->setVisible(geodesic_);
    advanced_->setVisible(!geodesic_);
    active_=true;dock_->setWindowTitle(geodesic_ ? "测地线弧长":"圆柱弧长");dock_->show();dock_->raise();updateUi();return true;
}
void ArcMeasurementTool::stop()
{
    if (!active_) return;
    if (busy_) emit cancelRequested();active_=busy_=picking_=cancelling_=false;dock_->hide();emit closed();
}
void ArcMeasurementTool::setBusy(bool busy,bool picking) { busy_=busy;picking_=picking;cancelling_=false;updateUi(); }
bool ArcMeasurementTool::usesPickedPosition() const { return position_->currentIndex()==1; }
std::string ArcMeasurementTool::cloudName() const { return cloud_->currentText().toStdString(); }
std::string ArcMeasurementTool::cylinderName() const { return reference_->currentText().toStdString(); }
bool ArcMeasurementTool::parameters(MeasureArc::ArcParams& params,double& radius)
{
    for (auto field:fields_) field->setStyleSheet({});
    auto read=[&](int index,double& value) { bool ok;value=fields_[index]->text().toDouble(&ok);if (!ok || !std::isfinite(value)) { fields_[index]->setStyleSheet("border: 1px solid #dd625d");fields_[index]->setFocus();return false; }return true; };
    if (geodesic_) return read(7,radius) && radius>0;
    params.method=method_->currentIndex()==0 ? MeasureArc::CARDINAL_SPLINE:MeasureArc::BSPLINE_LSQ;
    if (!read(0,params.sliceThicknessFactor) || !read(1,params.integrationTolerance) || params.sliceThicknessFactor<=0 || params.integrationTolerance<=0) return false;
    double count,degree;
    if (params.method==MeasureArc::CARDINAL_SPLINE) {
        if (!read(2,count) || count<2 || count>1000000 || count!=std::floor(count) || !read(3,params.virtualPointExtrapolation) || params.virtualPointExtrapolation<0) return false;
        params.downsampleTargetSize=static_cast<int>(count);
    } else {
        if (!read(4,degree) || !read(5,count) || !read(6,params.bsplineSmoothingFactor) || degree<1 || count<=degree || count>1000000 || count!=std::floor(count) || degree!=std::floor(degree) || params.bsplineSmoothingFactor<0) return false;
        params.bsplineDegree=static_cast<int>(degree);params.bsplineControlPoints=static_cast<int>(count);
    }
    return true;
}
void ArcMeasurementTool::updateUi()
{
    for (auto combo:{cloud_,reference_,method_,position_}) combo->setEnabled(!busy_);advanced_->setEnabled(!busy_);fields_[7]->setEnabled(!busy_);
    measure_->setEnabled(active_ && !busy_ && cloud_->currentIndex()>=0 && (geodesic_ || reference_->currentIndex()>=0));
    measure_->setText(busy_ ? (picking_ ? "选取位置":"正在测量"):(geodesic_ ? "选取两点":"开始测量"));
    cancel_->setEnabled(busy_ && !cancelling_);cancel_->setText(cancelling_ ? "正在取消":"取消计算");clear_->setEnabled(!busy_);
}
bool ArcMeasurementTool::eventFilter(QObject* watched,QEvent* event)
{
    if (!active_) return false;
    if (event->type()==QEvent::Close && (watched==dock_ || watched==view_->window())) stop();
    if (event->type()==QEvent::KeyPress) {
        auto widget=qobject_cast<QWidget*>(watched);if (!widget || (widget->window()!=view_->window() && widget->window()!=dock_->window())) return false;
        auto key=static_cast<QKeyEvent*>(event);
        if (key->key()==Qt::Key_Escape && (!picking_ || widget==dock_ || dock_->isAncestorOf(widget))) { stop();return true; }
        if ((key->key()==Qt::Key_Enter || key->key()==Qt::Key_Return) && key->modifiers()==Qt::ControlModifier && measure_->isEnabled()) { emit measureRequested();return true; }
    }
    return false;
}
