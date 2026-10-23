#include "Basic/ViewportAnnotationOverlay.h"
#include "Basic/ViewportResultOverlay.h"
#include <QEvent>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

ViewportAnnotationOverlay::ViewportAnnotationOverlay(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results, bool filledMarkers)
    : QWidget(view), renderer_(renderer), results_(results), filledMarkers_(filledMarkers)
{
    setObjectName("viewportAnnotationOverlay");
    selectorBar_=new QWidget(view);selectorBar_->setObjectName("viewportAnnotationSelector");
    auto layout=new QVBoxLayout(selectorBar_);layout->setContentsMargins(8,4,8,4);layout->setSpacing(3);
    auto row=new QHBoxLayout();row->addWidget(new QLabel("查看标注",selectorBar_));selector_=new QComboBox(selectorBar_);row->addWidget(selector_,1);layout->addLayout(row);
    selectedDetails_=new QLabel(selectorBar_);selectedDetails_->setObjectName("annotationSelectedDetails");selectedDetails_->setWordWrap(true);selectedDetails_->setTextInteractionFlags(Qt::TextSelectableByMouse);layout->addWidget(selectedDetails_);
    selectorBar_->setStyleSheet("QWidget { color: #edf5f7; background: #0a1e29; } QComboBox { border: 1px solid #42616d; border-radius: 0; padding: 2px; } QAbstractItemView { color: #edf5f7; background: #0a1e29; }");
    auto selectorFont=font();selectorFont.setPixelSize(13);selectorBar_->setFont(selectorFont);
    connect(selector_,&QComboBox::currentIndexChanged,this,[this](int index) {
        if (index>=0 && index<static_cast<int>(annotations_.size())) { selectedDetails_->setText(annotations_[index].title+"\n"+annotations_[index].detail);selector_->setToolTip(selectedDetails_->text()); }
        update();
    });
    selectorBar_->hide();
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    setGeometry(view->rect());
    view->installEventFilter(this);
    callback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    callback_->SetClientData(this);
    callback_->SetCallback([](vtkObject*, unsigned long, void* data, void*) {
        auto overlay = static_cast<ViewportAnnotationOverlay*>(data);
        if (overlay->isVisible()) overlay->update();
    });
    observer_ = renderer_->AddObserver(vtkCommand::EndEvent, callback_);
    hide();
}

ViewportAnnotationOverlay::~ViewportAnnotationOverlay()
{
    renderer_->RemoveObserver(observer_);
    delete selectorBar_;
}

void ViewportAnnotationOverlay::setAnnotations(std::vector<Annotation> annotations)
{
    const auto previous=selector_->currentText();
    annotations_ = std::move(annotations);
    {
        QSignalBlocker block(selector_);selector_->clear();
        for (const auto& annotation:annotations_) selector_->addItem(annotation.title);
        int selected=selector_->findText(previous);
        if (selected<0) {
            selected=0;
            for (size_t i=0;i<annotations_.size();++i) if (!annotations_[i].primary) { selected=static_cast<int>(i);break; }
        }
        selector_->setCurrentIndex(selected);
    }
    if (annotations_.empty()) { selectorBar_->hide();cards_.clear();selective_=false;selectedDetails_->clear(); }
    else { const auto index=std::max(0,selector_->currentIndex());selectedDetails_->setText(annotations_[index].title+"\n"+annotations_[index].detail); }
    setGeometry(parentWidget()->rect());
    setVisible(!annotations_.empty());
    lower();
    update();
}

void ViewportAnnotationOverlay::clear()
{
    annotations_.clear();cards_.clear();selective_=false;selectorBar_->hide();selector_->clear();
    hide();
}

void ViewportAnnotationOverlay::avoidWidget(QWidget* widget) { obstacle_ = widget; }

bool ViewportAnnotationOverlay::eventFilter(QObject* watched, QEvent* event)
{
    if (watched==parentWidget() && event->type()==QEvent::Hide) selectorBar_->hide();
    if (watched == parentWidget() && event->type() == QEvent::Resize) {
        setGeometry(parentWidget()->rect());
        update();
    }
    return false;
}

bool ViewportAnnotationOverlay::project(const Point& point, QPointF& screen) const
{
    const auto size = renderer_->GetRenderWindow()->GetSize();
    if (size[0] <= 0 || size[1] <= 0) return false;
    auto camera = renderer_->GetActiveCamera();
    const auto position = camera->GetPosition();
    const auto direction = camera->GetDirectionOfProjection();
    double forward = 0;
    for (int i = 0; i < 3; ++i) forward += (point[i] - position[i]) * direction[i];
    if (forward <= 0) return false;
    renderer_->SetWorldPoint(point[0], point[1], point[2], 1);
    renderer_->WorldToDisplay();
    const auto display = renderer_->GetDisplayPoint();
    screen = {display[0] * width() / size[0], height() - display[1] * height() / size[1]};
    return std::isfinite(screen.x()) && std::isfinite(screen.y()) && display[2] >= 0 && display[2] <= 1
        && rect().contains(screen.toPoint());
}

bool ViewportAnnotationOverlay::layoutCards(const std::vector<size_t>& indices, const QFont& title,
    const QFont& detail, bool compact, bool selector, std::vector<QRectF>& cards) const
{
    const QFontMetricsF titleMetrics(title),detailMetrics(detail);
    const QRectF available(10,10,width()-20,height()-20);
    std::vector<QRectF> occupied;
    if (results_ && !results_->occupiedRect().isEmpty()) occupied.push_back(QRectF(results_->occupiedRect()).adjusted(-8,-8,8,8));
    if (obstacle_ && obstacle_->isVisible()) occupied.push_back(obstacle_->geometry().adjusted(-8,-8,8,8));
    if (selector) occupied.push_back(selectorBar_->geometry().adjusted(-6,-6,6,6));
    std::vector<QPointF> anchors;
    for (const auto& annotation:annotations_) { QPointF anchor;if (project(annotation.anchor,anchor)) anchors.push_back(anchor); }
    cards.assign(annotations_.size(),{});
    for (const auto index:indices) {
        const auto& annotation=annotations_[index];QPointF anchor;if (!project(annotation.anchor,anchor)) continue;
        const double cardWidth=std::min(available.width(),std::max(titleMetrics.horizontalAdvance(annotation.title),detailMetrics.horizontalAdvance(annotation.detail))+24);
        const QSizeF cardSize(cardWidth,compact ? 54:65);
        auto bounded=[&](double x,double y) {
            return QRectF(QPointF(std::clamp(x,available.left(),available.right()-cardSize.width()),
                                  std::clamp(y,available.top(),available.bottom()-cardSize.height())),cardSize);
        };
        auto fits=[&](const QRectF& candidate,double gap) {
            return std::none_of(occupied.begin(),occupied.end(),[&](const QRectF& other) { return other.intersects(candidate.adjusted(-gap,-gap,gap,gap)); })
                && std::none_of(anchors.begin(),anchors.end(),[&](const QPointF& point) { return candidate.adjusted(-8,-8,8,8).contains(point); });
        };
        QRectF card;
        bool placed=false;
        for (double dy:{-38.0,22.0,-110.0,94.0,-182.0,166.0}) {
            for (double x:{anchor.x()+32,anchor.x()-cardWidth-32}) {
                const auto candidate=bounded(x,anchor.y()+dy);
                if (fits(candidate,6)) { card=candidate;placed=true;break; }
            }
            if (placed) break;
        }
        if (!placed) {
            double best=std::numeric_limits<double>::max();
            for (double y=available.top();y+cardSize.height()<=available.bottom();y+=8) {
                for (double x=available.left();x+cardSize.width()<=available.right();x+=8) {
                    const QRectF candidate(QPointF(x,y),cardSize);if (!fits(candidate,4)) continue;
                    const auto delta=candidate.center()-anchor;const double distance=delta.x()*delta.x()+delta.y()*delta.y();
                    if (distance<best) { best=distance;card=candidate;placed=true; }
                }
            }
        }
        if (!placed) { cards.clear();return false; }
        cards[index]=card;occupied.push_back(card);
    }
    return true;
}

void ViewportAnnotationOverlay::paintEvent(QPaintEvent*)
{
    if (annotations_.empty() || width()<100 || height()<100) { selectorBar_->hide();cards_.clear();return; }
    const bool compact=width()<650 && annotations_.size()>2;
    QFont titleFont=font(),detailFont=font();titleFont.setPixelSize(compact ? 14:16);titleFont.setBold(true);detailFont.setPixelSize(compact ? 12:13);
    std::vector<size_t> indices(annotations_.size());std::iota(indices.begin(),indices.end(),0);
    selective_=!layoutCards(indices,titleFont,detailFont,compact,false,cards_);
    if (selective_) {
        selectorBar_->setGeometry(12,height()-106,std::min(380,width()-24),94);selectorBar_->show();selectorBar_->raise();
        indices.clear();const auto selected=static_cast<size_t>(std::max(0,selector_->currentIndex()));
        for (size_t i=0;i<annotations_.size();++i) if (annotations_[i].primary || i==selected) indices.push_back(i);
        if (!layoutCards(indices,titleFont,detailFont,compact,true,cards_)) {
            if (!layoutCards({selected},titleFont,detailFont,compact,true,cards_)) cards_.assign(annotations_.size(),{});
        }
    } else selectorBar_->hide();
    QPainter painter(this);painter.setRenderHint(QPainter::Antialiasing);
    const QFontMetricsF titleMetrics(titleFont),detailMetrics(detailFont);
    for (size_t i=0;i<annotations_.size();++i) {
        if (i>=cards_.size() || cards_[i].isEmpty()) continue;
        const auto& annotation=annotations_[i];const auto& card=cards_[i];QPointF anchor;if (!project(annotation.anchor,anchor)) continue;
        const QPointF end(anchor.x()<card.center().x() ? card.left():card.right(),card.center().y());
        QPainterPath leader(anchor);leader.lineTo(QPointF(end.x()+(end.x()>anchor.x() ? -12:12),anchor.y()));leader.lineTo(end);
        painter.setBrush(Qt::NoBrush);painter.setPen(QPen(QColor("#08222d"),4.5,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));painter.drawPath(leader);
        painter.setPen(QPen(annotation.color,1.8,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));painter.drawPath(leader);
        painter.setPen(QPen(annotation.color,1));painter.setBrush(ViewportResultOverlay::backgroundColor());painter.drawRect(card);
        painter.setFont(titleFont);painter.setPen(Qt::white);
        painter.drawText(card.adjusted(12,compact ? 6:9,-12,compact ? -27:-32),Qt::AlignVCenter|Qt::AlignLeft,titleMetrics.elidedText(annotation.title,Qt::ElideRight,card.width()-24));
        painter.setFont(detailFont);painter.setPen(QColor("#c4d6dc"));
        painter.drawText(card.adjusted(12,compact ? 28:34,-12,compact ? -5:-8),Qt::AlignVCenter|Qt::AlignLeft,detailMetrics.elidedText(annotation.detail,Qt::ElideRight,card.width()-24));
    }
    auto markerFont=font();markerFont.setPixelSize(12);markerFont.setBold(true);
    for (size_t i=0;i<annotations_.size();++i) {
        const auto& annotation=annotations_[i];QPointF anchor;if (!project(annotation.anchor,anchor)) continue;
        for (const auto& point:annotation.endpoints) {
            QPointF endpoint;if (!project(point,endpoint)) continue;
            painter.setPen(QPen(QColor("#08222d"),3));painter.setBrush(annotation.color);painter.drawEllipse(endpoint,4.5,4.5);
        }
        const double radius=selective_ && static_cast<int>(i)==selector_->currentIndex() ? 6:4.5;
        painter.setPen(QPen(annotation.color,2.5));painter.setBrush(filledMarkers_ ? QBrush(Qt::white):QBrush(Qt::NoBrush));painter.drawEllipse(anchor,radius,radius);
        if (!annotation.marker.isEmpty()) {
            QPainterPath number;number.addText(anchor+QPointF(8,-8),markerFont,annotation.marker);
            painter.setPen(QPen(QColor("#08222d"),3));painter.setBrush(annotation.color);painter.drawPath(number);
            painter.setPen(Qt::NoPen);painter.drawPath(number);
        }
    }
}
