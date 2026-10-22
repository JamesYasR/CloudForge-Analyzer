#include "Basic/WeldPreparationOverlay.h"
#include <QEvent>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QStringList>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <algorithm>
#include <cmath>

WeldPreparationOverlay::WeldPreparationOverlay(QWidget* view, vtkRenderer* renderer)
    : QWidget(view), renderer_(renderer)
{
    setObjectName("weldPreparationOverlay");
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    setGeometry(view->rect());
    view->installEventFilter(this);
    callback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    callback_->SetClientData(this);
    callback_->SetCallback([](vtkObject*, unsigned long, void* data, void*) {
        auto overlay = static_cast<WeldPreparationOverlay*>(data);
        if (overlay->isVisible()) overlay->update();
    });
    observer_ = renderer_->AddObserver(vtkCommand::EndEvent, callback_);
    hide();
}

WeldPreparationOverlay::~WeldPreparationOverlay()
{
    renderer_->RemoveObserver(observer_);
}

void WeldPreparationOverlay::setAnnotations(std::vector<Annotation> annotations)
{
    annotations_ = std::move(annotations);
    setGeometry(parentWidget()->rect());
    setVisible(!annotations_.empty());
    lower();
    update();
}

void WeldPreparationOverlay::clear()
{
    annotations_.clear();
    hide();
}

bool WeldPreparationOverlay::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == parentWidget() && event->type() == QEvent::Resize) {
        setGeometry(parentWidget()->rect());
        update();
    }
    return false;
}

bool WeldPreparationOverlay::project(const Point& point, QPointF& screen) const
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

void WeldPreparationOverlay::paintEvent(QPaintEvent*)
{
    if (annotations_.empty() || width() < 100 || height() < 100) return;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QFont titleFont = font(), detailFont = font();
    titleFont.setPixelSize(16);
    titleFont.setBold(true);
    detailFont.setPixelSize(13);
    const QFontMetricsF titleMetrics(titleFont), detailMetrics(detailFont);
    const QRectF available(10, 10, width() - 20, height() - 20);
    const QRectF legend(12, 12, std::min(320, width() - 24), 66);
    painter.setPen(QPen(QColor("#42616d"), 1));
    painter.setBrush(QColor(10, 30, 41, 238));
    painter.drawRoundedRect(legend, 8, 8);
    painter.setFont(detailFont);
    const std::array<QColor, 4> colors{QColor("#42f294"), QColor("#ffda55"), QColor("#68b7ff"), QColor("#ff6b65")};
    const QStringList names{"近件缝边", "远件缝边", "径向阶差", "轴向间隙"};
    for (int i = 0; i < 4; ++i) {
        const QPointF pos(legend.left() + 16 + i % 2 * 153, legend.top() + 21 + i / 2 * 25);
        painter.setPen(QPen(colors[i], 3));
        painter.drawLine(pos, pos + QPointF(16, 0));
        painter.setPen(QColor("#edf5f7"));
        painter.drawText(pos + QPointF(24, 4), names[i]);
    }
    std::vector<QRectF> occupied{legend.adjusted(-8, -8, 8, 8)};
    std::vector<QPointF> anchors;
    for (const auto& annotation : annotations_) {
        QPointF anchor;
        if (project(annotation.anchor, anchor)) anchors.push_back(anchor);
    }
    for (const auto& annotation : annotations_) {
        QPointF anchor;
        if (!project(annotation.anchor, anchor)) continue;
        const double cardWidth = std::min(available.width(), std::max(titleMetrics.horizontalAdvance(annotation.title),
                                                                 detailMetrics.horizontalAdvance(annotation.detail)) + 34);
        const QSizeF cardSize(cardWidth, 65);
        auto bounded = [&](double x, double y) {
            return QRectF(QPointF(std::clamp(x, available.left(), available.right() - cardSize.width()),
                                  std::clamp(y, available.top(), available.bottom() - cardSize.height())), cardSize);
        };
        QRectF card;
        bool placed = false;
        for (double dy : {-38.0, 22.0, -110.0, 94.0, -182.0, 166.0}) {
            for (double x : {anchor.x() + 32, anchor.x() - cardWidth - 32}) {
                const auto candidate = bounded(x, anchor.y() + dy);
                if (std::any_of(anchors.begin(), anchors.end(), [&](const QPointF& point) {
                    return candidate.adjusted(-10, -10, 10, 10).contains(point);
                })) continue;
                if (std::none_of(occupied.begin(), occupied.end(), [&](const QRectF& other) {
                    return other.intersects(candidate.adjusted(-6, -6, 6, 6));
                })) { card = candidate; placed = true; break; }
            }
            if (placed) break;
        }
        if (!placed) card = bounded(anchor.x() + 32, available.bottom() - cardSize.height());
        occupied.push_back(card);
        const QPointF end(anchor.x() < card.center().x() ? card.left() : card.right(), card.center().y());
        QPainterPath leader(anchor);
        leader.lineTo(QPointF(end.x() + (end.x() > anchor.x() ? -12 : 12), anchor.y()));
        leader.lineTo(end);
        painter.setBrush(Qt::NoBrush);
        painter.setPen(QPen(QColor("#08222d"), 4.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPath(leader);
        painter.setPen(QPen(annotation.color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPath(leader);
        for (const auto& point : annotation.endpoints) {
            QPointF endpoint;
            if (!project(point, endpoint)) continue;
            painter.setPen(QPen(QColor("#08222d"), 3));
            painter.setBrush(annotation.color);
            painter.drawEllipse(endpoint, 4.5, 4.5);
        }
        painter.setPen(QPen(annotation.color, 2.5));
        painter.setBrush(QColor("#ffffff"));
        painter.drawEllipse(anchor, 4.5, 4.5);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 8, 14, 65));
        painter.drawRoundedRect(card.translated(2, 3), 8, 8);
        painter.setPen(QPen(annotation.color, 1.3));
        painter.setBrush(QColor(10, 30, 41, 245));
        painter.drawRoundedRect(card, 8, 8);
        painter.setPen(QPen(annotation.color, 3, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(card.topLeft() + QPointF(10, 13), card.bottomLeft() + QPointF(10, -13));
        painter.setFont(titleFont);
        painter.setPen(QColor("#ffffff"));
        painter.drawText(card.adjusted(21, 9, -8, -32), Qt::AlignVCenter | Qt::AlignLeft, annotation.title);
        painter.setFont(detailFont);
        painter.setPen(QColor("#c4d6dc"));
        painter.drawText(card.adjusted(21, 34, -8, -8), Qt::AlignVCenter | Qt::AlignLeft, annotation.detail);
    }
}
