#include "Basic/ViewportColorLegend.h"
#include "Basic/ViewportResultOverlay.h"
#include <QComboBox>
#include <QEvent>
#include <QFontMetrics>
#include <QPainter>
#include <QSignalBlocker>
#include <algorithm>
#include <cmath>

ViewportColorLegend::ViewportColorLegend(QWidget* view):QWidget(view)
{
    setObjectName("viewportColorLegend");
    selector_=new QComboBox(this);selector_->setObjectName("colorLegendSelector");
    selector_->setStyleSheet("QComboBox { color: #edf5f7; background: #0a1e29; border: 1px solid #42616d; border-radius: 0; padding: 3px; } QAbstractItemView { color: #edf5f7; background: #0a1e29; }");
    auto selectorFont=font();selectorFont.setPixelSize(14);selector_->setFont(selectorFont);
    connect(selector_,&QComboBox::currentIndexChanged,this,[this](int index) { if (index>=0 && index<static_cast<int>(entries_.size())) setToolTip(entries_[index].title);update(); });
    view->installEventFilter(this);hide();
}
QGradientStops ViewportColorLegend::sequential() { return {{0,Qt::green},{.5,Qt::yellow},{1,Qt::red}}; }
QGradientStops ViewportColorLegend::signedHeight() { return {{0,Qt::blue},{.25,Qt::cyan},{.5,Qt::green},{.75,Qt::yellow},{1,Qt::red}}; }
QGradientStops ViewportColorLegend::signedSurface() { return {{0,Qt::blue},{.5,Qt::white},{1,Qt::red}}; }
void ViewportColorLegend::setEntry(Entry entry)
{
    if (!std::isfinite(entry.minimum) || !std::isfinite(entry.maximum) || entry.maximum<entry.minimum || entry.colors.empty()) return;
    auto found=std::find_if(entries_.begin(),entries_.end(),[&](const Entry& item) { return item.id==entry.id; });
    const int selected=found==entries_.end() ? static_cast<int>(entries_.size()):static_cast<int>(found-entries_.begin());
    if (found==entries_.end()) entries_.push_back(std::move(entry));else *found=std::move(entry);
    { QSignalBlocker block(selector_);selector_->clear();for (const auto& item:entries_) selector_->addItem(item.title);selector_->setCurrentIndex(selected); }
    setToolTip(entries_[selected].title);updateLayout();update();
}
void ViewportColorLegend::removeEntry(const QString& id)
{
    entries_.erase(std::remove_if(entries_.begin(),entries_.end(),[&](const Entry& item) { return item.id==id; }),entries_.end());
    { QSignalBlocker block(selector_);selector_->clear();for (const auto& item:entries_) selector_->addItem(item.title); }
    updateLayout();update();
}
void ViewportColorLegend::clear() { entries_.clear();selector_->clear();hide(); }
bool ViewportColorLegend::eventFilter(QObject* watched,QEvent* event)
{
    if (watched==parentWidget() && event->type()==QEvent::Resize) updateLayout();
    return false;
}
void ViewportColorLegend::updateLayout()
{
    const bool compact=parentWidget()->width()<650;
    int preferredWidth=compact ? 116:172;
    if (compact && !entries_.empty()) {
        const auto& entry=entries_[std::max(0,selector_->currentIndex())];
        auto labels=font();labels.setPixelSize(16);const QFontMetrics metrics(labels);
        for (double value:{entry.minimum,entry.maximum,(entry.minimum+entry.maximum)/2})
            preferredWidth=std::max(preferredWidth,metrics.horizontalAdvance(QString::number(value,'g',5))+58);
    }
    const int w=std::min(preferredWidth,std::max(1,parentWidget()->width()-24));
    const int h=std::min(compact ? 232:292,std::max(1,parentWidget()->height()-24));
    setGeometry(parentWidget()->width()-w-12,std::max(12,(parentWidget()->height()-h)/2),w,h);
    selector_->setGeometry(10,9,w-20,30);selector_->setVisible(entries_.size()>1);
    setVisible(!entries_.empty() && w>=100 && h>=150);raise();
}
void ViewportColorLegend::paintEvent(QPaintEvent*)
{
    const int selected=std::max(0,selector_->currentIndex());if (selected>=static_cast<int>(entries_.size())) return;
    const auto& entry=entries_[selected];
    QPainter painter(this);painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(ViewportResultOverlay::borderColor());painter.setBrush(ViewportResultOverlay::backgroundColor());painter.drawRect(rect().adjusted(0,0,-1,-1));
    auto title=font();title.setPixelSize(14);title.setBold(true);painter.setFont(title);painter.setPen(ViewportResultOverlay::textColor());
    if (entries_.size()==1) painter.drawText(QRect(12,9,width()-24,25),Qt::AlignLeft|Qt::AlignVCenter,QFontMetrics(title).elidedText(entry.title,Qt::ElideRight,width()-24));
    auto labels=font();labels.setPixelSize(16);painter.setFont(labels);
    painter.drawText(QRect(12,40,width()-24,20),Qt::AlignLeft|Qt::AlignVCenter,"mm");
    const QRectF bar(18,74,18,height()-98);
    QLinearGradient gradient(bar.bottomLeft(),bar.topLeft());gradient.setStops(entry.colors);
    painter.fillRect(bar,entry.minimum==entry.maximum ? QBrush(entry.colors.front().second):QBrush(gradient));
    painter.setPen(ViewportResultOverlay::borderColor());painter.setBrush(Qt::NoBrush);painter.drawRect(bar);
    const int count=entry.minimum==entry.maximum ? 1:bar.height()>=140 ? 5:bar.height()>=80 ? 3:2;
    for (int i=0;i<count;++i) {
        const double t=count==1 ? .5:i/static_cast<double>(count-1);
        const double value=entry.minimum+t*(entry.maximum-entry.minimum),y=bar.bottom()-t*bar.height();
        QString text=QString::number(std::abs(value)<1e-12 ? 0:value, 'g', 5);
        painter.setPen(QColor("#abc5d1"));painter.drawLine(QPointF(bar.right()+2,y),QPointF(bar.right()+7,y));
        painter.setPen(ViewportResultOverlay::textColor());painter.drawText(QRectF(48,y-11,width()-56,22),Qt::AlignLeft|Qt::AlignVCenter,text);
    }
}
