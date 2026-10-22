#include "Basic/WeldParallelismTool.h"
#include "Basic/ViewportResultOverlay.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
const QColor colors[2]={QColor("#56c4ff"),QColor("#ffd166")};
vtkSmartPointer<vtkActor> actorFor(vtkPolyData* data)
{
    auto mapper=vtkSmartPointer<vtkPolyDataMapper>::New();mapper->SetInputData(data);
    auto actor=vtkSmartPointer<vtkActor>::New();actor->SetMapper(mapper);actor->PickableOff();actor->UseBoundsOff();
    actor->GetProperty()->LightingOff();return actor;
}
}

class WeldParallelismPlot : public QWidget
{
public:
    explicit WeldParallelismPlot(QWidget* parent):QWidget(parent) { setObjectName("weldParallelPlot");setMinimumHeight(240);setMouseTracking(true);setToolTip("散点为焊缝点，实线为拟合走向；滚轮缩放，拖动平移，双击复位"); }
    void setResult(std::shared_ptr<MeasureWeldParallelism::Result> result) { result_=std::move(result);reset(); }
    void reset() { zoom_=1;pan_=QPointF();setProperty("hasData",bool(result_));update(); }
protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(),ViewportResultOverlay::backgroundColor());
        painter.setPen(ViewportResultOverlay::borderColor());painter.drawRect(rect().adjusted(0,0,-1,-1));
        painter.setPen(ViewportResultOverlay::textColor());
        if (!result_) { painter.drawText(rect(),Qt::AlignCenter,"展开图");return; }
        const QRectF area=QRectF(rect()).adjusted(52,34,-14,-38);
        Eigen::Vector2d low=Eigen::Vector2d::Constant(std::numeric_limits<double>::max()),high=-low;
        for (const auto& cloud:result_->projected) for (const auto& p:cloud) { low=low.cwiseMin(p);high=high.cwiseMax(p); }
        const auto middle=(low+high)*.5;
        const double scale=zoom_*std::min(area.width()/std::max(high.x()-low.x(),1.0),area.height()/std::max(high.y()-low.y(),1.0))*.88;
        setProperty("xPixelsPerMm",scale);setProperty("yPixelsPerMm",scale);
        auto map=[&](const Eigen::Vector2d& p) { return area.center()+pan_+QPointF((p.x()-middle.x())*scale,-(p.y()-middle.y())*scale); };
        painter.save();painter.setClipRect(area);
        const double desired=55/scale,power=std::pow(10,std::floor(std::log10(desired)));
        const double unit=desired/power,step=(unit<2 ? 2:unit<5 ? 5:10)*power;
        painter.setPen(QPen(QColor("#29454e"),1));
        const double xmin=middle.x()+(-area.width()/2-pan_.x())/scale,xmax=middle.x()+(area.width()/2-pan_.x())/scale;
        const double ymin=middle.y()+(-area.height()/2+pan_.y())/scale,ymax=middle.y()+(area.height()/2+pan_.y())/scale;
        for (double x=std::ceil(xmin/step)*step;x<=xmax;x+=step) { const auto p=map({x,middle.y()});painter.drawLine(QPointF(p.x(),area.top()),QPointF(p.x(),area.bottom())); }
        for (double y=std::ceil(ymin/step)*step;y<=ymax;y+=step) { const auto p=map({middle.x(),y});painter.drawLine(QPointF(area.left(),p.y()),QPointF(area.right(),p.y())); }
        for (int side=0;side<2;++side) {
            QColor faded=colors[side];faded.setAlpha(50);painter.setPen(QPen(faded,1.5));
            for (const auto& p:result_->projected[side]) painter.drawPoint(map(p));
            painter.setPen(QPen(colors[side],1,Qt::DotLine));
            const auto& centers=result_->centers[side];
            for (size_t i=1;i<centers.size();++i) painter.drawLine(map(centers[i-1]),map(centers[i]));
            const auto& c=result_->linePoint[side];const auto& u=result_->lineDirection[side];
            double a=std::numeric_limits<double>::max(),b=-a;
            for (const auto& p:centers) { a=std::min(a,(p-c).dot(u));b=std::max(b,(p-c).dot(u)); }
            painter.setPen(QPen(ViewportResultOverlay::backgroundColor(),5));painter.drawLine(map(c+a*u),map(c+b*u));
            painter.setPen(QPen(colors[side],2.5));painter.drawLine(map(c+a*u),map(c+b*u));
        }
        painter.restore();painter.setPen(QColor("#abc5d1"));
        for (double x=std::ceil(xmin/step)*step;x<=xmax;x+=step) { const auto p=map({x,middle.y()});painter.drawText(QRectF(p.x()-25,area.bottom()+2,50,18),Qt::AlignCenter,QString::number(x,'f',step<1?1:0)); }
        for (double y=std::ceil(ymin/step)*step;y<=ymax;y+=step) { const auto p=map({middle.x(),y});painter.drawText(QRectF(1,p.y()-9,47,18),Qt::AlignRight|Qt::AlignVCenter,QString::number(y,'f',step<1?1:0)); }
        painter.drawText(QRectF(area.left(),height()-19,area.width(),18),Qt::AlignCenter,"周向弧长 (mm)");
        painter.save();painter.translate(13,area.center().y());painter.rotate(-90);painter.drawText(QRectF(-area.height()/2,-9,area.height(),18),Qt::AlignCenter,"轴向位置 (mm)");painter.restore();
        for (int side=0;side<2;++side) { const int x=12+side*105;painter.setPen(QPen(colors[side],2));painter.drawLine(x,16,x+17,16);painter.setPen(ViewportResultOverlay::textColor());painter.drawText(x+24,21,QString("焊缝 %1").arg(side+1)); }
    }
    void wheelEvent(QWheelEvent* event) override { zoom_=std::clamp(zoom_*std::pow(1.0015,event->angleDelta().y()),.3,15.0);update();event->accept(); }
    void mousePressEvent(QMouseEvent* event) override { if (event->button()==Qt::LeftButton) { dragging_=true;last_=event->position(); } }
    void mouseMoveEvent(QMouseEvent* event) override { if (dragging_) { pan_+=event->position()-last_;last_=event->position();update(); } }
    void mouseReleaseEvent(QMouseEvent*) override { dragging_=false; }
    void mouseDoubleClickEvent(QMouseEvent*) override { reset(); }
private:
    std::shared_ptr<MeasureWeldParallelism::Result> result_;
    double zoom_=1;
    QPointF pan_,last_;
    bool dragging_=false;
};

WeldParallelismTool::WeldParallelismTool(QWidget* view,vtkRenderer* renderer,ViewportResultOverlay* results)
    :view_(view),renderer_(renderer),results_(results) { buildPanel();qApp->installEventFilter(this); }
WeldParallelismTool::~WeldParallelismTool() { stop();qApp->removeEventFilter(this);delete dock_; }
void WeldParallelismTool::buildPanel()
{
    dock_=new QDockWidget("焊缝平行度",view_->window());dock_->setObjectName("weldParallelDock");
    dock_->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
    auto body=new QWidget(dock_);body->setMinimumWidth(330);
    body->setStyleSheet("QPushButton { border: 1px solid palette(mid); padding: 6px 8px; border-radius: 0; } QPushButton:hover { background: palette(light); } QPushButton:disabled { color: palette(mid); }");
    auto layout=new QVBoxLayout(body);auto form=new QFormLayout();
    reference_=new QComboBox(body);reference_->setObjectName("weldParallelReference");reference_->setPlaceholderText("请选择拟合圆柱");
    first_=new QComboBox(body);first_->setObjectName("weldParallelFirst");second_=new QComboBox(body);second_->setObjectName("weldParallelSecond");
    form->addRow("参考圆柱",reference_);form->addRow("焊缝 1",first_);form->addRow("焊缝 2",second_);layout->addLayout(form);
    fit_=new QPushButton("三阶段拟合圆柱",body);fit_->setObjectName("weldParallelFit");layout->addWidget(fit_);
    measure_=new QPushButton("开始测量",body);measure_->setObjectName("weldParallelMeasure");measure_->setToolTip("开始测量（Ctrl+Enter）");
    cancel_=new QPushButton("取消计算",body);cancel_->setObjectName("weldParallelCancel");
    auto buttons=new QHBoxLayout();buttons->addWidget(measure_);buttons->addWidget(cancel_);layout->addLayout(buttons);
    summary_=new QLabel(body);summary_->setObjectName("weldParallelSummary");summary_->setWordWrap(true);layout->addWidget(summary_);
    plot_=new WeldParallelismPlot(body);layout->addWidget(plot_,1);
    auto reset=new QPushButton("复位展开图",body);reset->setObjectName("weldParallelReset");layout->addWidget(reset);
    auto display=new QHBoxLayout();points_=new QCheckBox("焊缝点",body);points_->setObjectName("weldParallelPoints");points_->setChecked(true);
    curves_=new QCheckBox("拟合走向",body);curves_->setObjectName("weldParallelCurves");curves_->setChecked(true);
    display->addWidget(points_);display->addWidget(curves_);layout->addLayout(display);
    clear_=new QPushButton("清除结果",body);clear_->setObjectName("weldParallelClear");layout->addWidget(clear_);
    auto exit=new QPushButton("结束测量",body);exit->setObjectName("weldParallelExit");exit->setToolTip("结束测量（Esc / Q）");layout->addWidget(exit);
    dock_->setWidget(body);
    if (auto main=qobject_cast<QMainWindow*>(view_->window())) { main->addDockWidget(Qt::LeftDockWidgetArea,dock_);main->resizeDocks({dock_},{370},Qt::Horizontal); }
    dock_->hide();
    connect(measure_,&QPushButton::clicked,this,&WeldParallelismTool::measureRequested);
    connect(cancel_,&QPushButton::clicked,this,[this]() { setCancelling();emit cancelRequested(); });
    connect(fit_,&QPushButton::clicked,this,&WeldParallelismTool::fitReferenceRequested);
    connect(clear_,&QPushButton::clicked,this,&WeldParallelismTool::clearResult);
    connect(exit,&QPushButton::clicked,this,&WeldParallelismTool::stop);
    connect(reset,&QPushButton::clicked,plot_,&WeldParallelismPlot::reset);
    for (auto box:{points_,curves_}) connect(box,&QCheckBox::toggled,this,&WeldParallelismTool::updateVisibility);
    for (auto combo:{first_,second_,reference_}) connect(combo,&QComboBox::currentIndexChanged,this,[this]() { if (active_ && !busy_) { ++ticket_;clearResult();updateUi(); } });
    dock_->installEventFilter(this);updateUi();
}

bool WeldParallelismTool::start(const std::vector<Target>& targets,const std::vector<Reference>& references,const std::string& preferred)
{
    stop();targets_.clear();references_=references;
    for (const auto& target:targets) if (target.actor && target.cloud && !target.cloud->empty()) targets_.push_back(target);
    if (targets_.empty()) return false;
    { QSignalBlocker a(first_),b(second_),c(reference_);first_->clear();second_->clear();reference_->clear();
        for (const auto& target:targets_) { first_->addItem(QString::fromStdString(target.name));second_->addItem(QString::fromStdString(target.name)); }
        second_->setCurrentIndex(targets_.size()>1 ? 1:0);
        for (int i=0;i<static_cast<int>(targets_.size());++i) {
            const auto name=QString::fromStdString(targets_[i].name).toLower();
            if (name.contains("焊缝1") || name.contains("weld1") || name.contains("weld_1")) first_->setCurrentIndex(i);
            if (name.contains("焊缝2") || name.contains("weld2") || name.contains("weld_2")) second_->setCurrentIndex(i);
        }
        for (const auto& reference:references_) reference_->addItem(QString::fromStdString(reference.name)+QString(" · R %1 mm").arg(reference.cylinder.radius,0,'f',1));
        for (int i=0;i<static_cast<int>(references_.size());++i) if (references_[i].name==preferred) reference_->setCurrentIndex(i);
    }
    active_=true;dock_->show();dock_->raise();updateUi();emit activeChanged(true);return true;
}
void WeldParallelismTool::stop()
{
    if (!active_ && !busy_ && actors_.empty() && hidden_.empty()) return;
    const bool wasActive=active_,wasBusy=busy_;active_=busy_=false;++ticket_;
    if (wasBusy) emit cancelRequested();
    clearResult();dock_->hide();targets_.clear();references_.clear();
    if (wasActive) emit activeChanged(false);
}
WeldParallelismTool::Request WeldParallelismTool::beginMeasurement()
{
    if (!active_ || busy_ || reference_->currentIndex()<0 || first_->currentIndex()==second_->currentIndex()) return {};
    clearResult();busy_=true;cancelling_=false;updateUi();
    return {targets_.at(first_->currentIndex()),targets_.at(second_->currentIndex()),references_.at(reference_->currentIndex()),++ticket_};
}
void WeldParallelismTool::setCancelling() { if (busy_) { cancelling_=true;updateUi(); } }
bool WeldParallelismTool::finish(std::shared_ptr<MeasureWeldParallelism::Result> result,unsigned long ticket)
{
    if (!active_ || !busy_ || ticket!=ticket_) return false;
    busy_=false;
    if (cancelling_ || !result || !result->success) {
        emit resultMessage(cancelling_ ? "焊缝平行度测量已取消":result ? QString::fromStdString(result->message):"不可测: 未能获得焊缝夹角");updateUi();return false;
    }
    result_=std::move(result);visualize();plot_->setResult(result_);
    summary_->setText(QString("展开面夹角 %1°\n比较长度 %2 mm").arg(result_->angleDegrees,0,'f',3).arg(result_->comparisonLength,0,'f',2));
    results_->setSection("weld_parallelism",{"焊缝平行度",{{"展开面夹角",QString::number(result_->angleDegrees,'f',3)+"°",{}},{"比较长度",QString::number(result_->comparisonLength,'f',2)+" mm",{}}}});
    updateUi();updateVisibility();return true;
}
void WeldParallelismTool::clearAnnotations() { results_->removeSection("weld_parallelism");plot_->setResult(nullptr);summary_->clear(); }
void WeldParallelismTool::clearResult()
{
    for (auto& actor:actors_) renderer_->RemoveActor(actor);actors_.clear();
    for (auto& pair:hidden_) pair.first->SetVisibility(pair.second);hidden_.clear();
    result_.reset();clearAnnotations();updateUi();render();
}
void WeldParallelismTool::visualize()
{
    for (int side=0;side<2;++side) {
        const auto& target=targets_.at(side==0 ? first_->currentIndex():second_->currentIndex());
        hidden_.push_back({target.actor,target.actor->GetVisibility()});target.actor->VisibilityOff();
        auto points=vtkSmartPointer<vtkPoints>::New();auto vertices=vtkSmartPointer<vtkCellArray>::New();
        for (const auto& p:*target.cloud) { const auto index=points->InsertNextPoint(p.x,p.y,p.z);vertices->InsertNextCell(1,&index); }
        auto data=vtkSmartPointer<vtkPolyData>::New();data->SetPoints(points);data->SetVerts(vertices);
        auto cloudActor=actorFor(data);cloudActor->GetProperty()->SetColor(colors[side].redF(),colors[side].greenF(),colors[side].blueF());cloudActor->GetProperty()->SetPointSize(4);cloudActor->GetProperty()->SetOpacity(.75);
        renderer_->AddActor(cloudActor);actors_.push_back(cloudActor);
        auto curvePoints=vtkSmartPointer<vtkPoints>::New();auto lines=vtkSmartPointer<vtkCellArray>::New();
        for (const auto& p:result_->curves[side]) {
            const auto d=p-result_->frame.axisPoint();const Eigen::Vector3d radial=d-d.dot(result_->frame.axisDirection())*result_->frame.axisDirection();
            const Eigen::Vector3d raised=p+.25*radial.normalized();curvePoints->InsertNextPoint(raised.data());
        }
        lines->InsertNextCell(static_cast<vtkIdType>(result_->curves[side].size()));
        for (vtkIdType i=0;i<curvePoints->GetNumberOfPoints();++i) lines->InsertCellPoint(i);
        auto curve=vtkSmartPointer<vtkPolyData>::New();curve->SetPoints(curvePoints);curve->SetLines(lines);
        auto curveActor=actorFor(curve);curveActor->GetProperty()->SetColor(colors[side].redF(),colors[side].greenF(),colors[side].blueF());curveActor->GetProperty()->SetLineWidth(4);
        renderer_->AddActor(curveActor);actors_.push_back(curveActor);
    }
}
void WeldParallelismTool::updateUi()
{
    for (auto combo:{first_,second_,reference_}) combo->setEnabled(!busy_);
    measure_->setEnabled(active_ && !busy_ && reference_->currentIndex()>=0 && first_->currentIndex()!=second_->currentIndex());
    measure_->setText(busy_ ? "正在测量":"开始测量");cancel_->setEnabled(busy_ && !cancelling_);cancel_->setText(busy_ && cancelling_ ? "正在取消":"取消计算");
    fit_->setEnabled(!busy_);clear_->setEnabled(!busy_ && !actors_.empty());
}
void WeldParallelismTool::updateVisibility()
{
    for (size_t i=0;i<actors_.size();++i) actors_[i]->SetVisibility(i%2 ? curves_->isChecked():points_->isChecked());
    for (auto& pair:hidden_) pair.first->SetVisibility(points_->isChecked() ? 0:pair.second);
    render();
}
bool WeldParallelismTool::eventFilter(QObject* watched,QEvent* event)
{
    if (!active_) return false;
    if (event->type()==QEvent::Close && (watched==dock_ || watched==view_->window())) stop();
    if (event->type()==QEvent::KeyPress) {
        auto widget=qobject_cast<QWidget*>(watched);
        if (widget && (widget->window()==view_->window() || widget->window()==dock_->window())) {
            auto key=static_cast<QKeyEvent*>(event);
            if (key->key()==Qt::Key_Escape || (watched==view_ && key->key()==Qt::Key_Q && key->modifiers()==Qt::NoModifier)) { stop();return true; }
            if ((key->key()==Qt::Key_Return || key->key()==Qt::Key_Enter) && key->modifiers()==Qt::ControlModifier && measure_->isEnabled()) { emit measureRequested();return true; }
        }
    }
    return false;
}
void WeldParallelismTool::render() { if (view_ && view_->isVisible()) { renderer_->GetRenderWindow()->Render();view_->update(); } }
