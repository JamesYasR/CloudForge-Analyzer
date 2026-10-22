#include "Basic/ViewportResultOverlay.h"
#include <QEvent>
#include <QFontMetrics>
#include <QPainter>
#include <algorithm>

ViewportResultOverlay::ViewportResultOverlay(QWidget* view) : QWidget(view)
{
    setObjectName("viewportResultOverlay");
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    view->installEventFilter(this);
    hide();
}

QColor ViewportResultOverlay::backgroundColor() { return QColor(10, 30, 41, 238); }
QColor ViewportResultOverlay::borderColor() { return QColor("#42616d"); }
QColor ViewportResultOverlay::textColor() { return QColor("#edf5f7"); }

void ViewportResultOverlay::setSection(const QString& id, Section section)
{
    if (section.rows.empty()) { removeSection(id); return; }
    auto it = std::find_if(sections_.begin(), sections_.end(), [&](const Entry& item) { return item.id == id; });
    if (it == sections_.end()) sections_.push_back({id, std::move(section)});
    else it->section = std::move(section);
    updateLayout();
}

void ViewportResultOverlay::removeSection(const QString& id)
{
    sections_.erase(std::remove_if(sections_.begin(), sections_.end(), [&](const Entry& item) {
        return item.id == id;
    }), sections_.end());
    updateLayout();
}

void ViewportResultOverlay::clear()
{
    sections_.clear();
    updateLayout();
}

QRect ViewportResultOverlay::occupiedRect() const
{
    return isHidden() ? QRect() : geometry();
}

void ViewportResultOverlay::avoidWidget(QWidget* widget)
{
    if (!widget || widget->parentWidget() != parentWidget()) return;
    if (std::find(obstacles_.begin(), obstacles_.end(), widget) != obstacles_.end()) return;
    obstacles_.push_back(widget);
    widget->installEventFilter(this);
    updateLayout();
}

void ViewportResultOverlay::updateLayout()
{
    const QRect previous = geometry();
    const auto view = parentWidget();
    resize(std::max(1, std::min(320, view->width() - 24)), height());
    int top = 12;
    for (const auto& widget : obstacles_) {
        if (widget && !widget->isHidden()) top = std::max(top, widget->geometry().bottom() + 12);
    }
    setGeometry(12, top, width(), layoutContents(nullptr));
    setVisible(!sections_.empty() && view->width() > 48 && view->height() > 48);
    update();
    view->update(previous.united(geometry()));
}

bool ViewportResultOverlay::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == parentWidget()) {
        if (event->type() == QEvent::Resize) updateLayout();
    }
    else if (event->type() == QEvent::Show || event->type() == QEvent::Hide
             || event->type() == QEvent::Resize || event->type() == QEvent::Move) updateLayout();
    return false;
}

int ViewportResultOverlay::layoutContents(QPainter* painter) const
{
    QFont labelFont = font(), valueFont = font(), titleFont = font();
    labelFont.setPixelSize(13);
    valueFont.setPixelSize(16);
    valueFont.setBold(true);
    titleFont.setPixelSize(14);
    titleFont.setBold(true);
    const QFontMetrics labels(labelFont), values(valueFont), titles(titleFont);
    const int usable = std::max(1, width() - 32);
    int y = 12;
    for (size_t i = 0; i < sections_.size(); ++i) {
        const auto& section = sections_[i].section;
        if (i != 0) {
            if (painter) {
                painter->setPen(borderColor());
                painter->drawLine(16, y + 3, width() - 16, y + 3);
            }
            y += 16;
        }
        if (!section.title.isEmpty()) {
            if (painter) {
                painter->setFont(titleFont);
                painter->setPen(textColor());
                painter->drawText(QRect(16, y, usable, titles.height()), Qt::AlignLeft | Qt::AlignVCenter,
                                  titles.elidedText(section.title, Qt::ElideRight, usable));
            }
            y += titles.height() + 8;
        }
        const int columns = section.columns > 1 && width() >= 300 ? 2 : 1;
        const int cellWidth = (usable - (columns - 1) * 12) / columns;
        for (size_t row = 0; row < section.rows.size(); row += columns) {
            int rowHeight = 25;
            for (int col = 0; col < columns && row + col < section.rows.size(); ++col) {
                const auto& item = section.rows[row + col];
                const int x = 16 + col * (cellWidth + 12);
                const int swatch = item.color.isValid() ? 24 : 0;
                const int valueWidth = values.horizontalAdvance(item.value);
                const bool stacked = !item.value.isEmpty()
                    && labels.horizontalAdvance(item.label) + valueWidth + swatch + 16 > cellWidth;
                const int itemHeight = stacked ? labels.height() + values.height() + 8 : 25;
                rowHeight = std::max(rowHeight, itemHeight);
                if (!painter) continue;
                if (swatch) {
                    painter->setPen(QPen(item.color, 3));
                    painter->drawLine(x, y + 12, x + 16, y + 12);
                }
                painter->setFont(labelFont);
                painter->setPen(textColor());
                const int labelWidth = std::max(1, cellWidth - swatch - (stacked || item.value.isEmpty() ? 0 : valueWidth + 12));
                painter->drawText(QRect(x + swatch, y, labelWidth, stacked ? labels.height() : 25),
                                  Qt::AlignLeft | Qt::AlignVCenter,
                                  labels.elidedText(item.label, Qt::ElideRight, labelWidth));
                if (!item.value.isEmpty()) {
                    painter->setFont(valueFont);
                    painter->setPen(Qt::white);
                    painter->drawText(QRect(x + swatch, stacked ? y + labels.height() + 4 : y,
                                           cellWidth - swatch, stacked ? values.height() : 25),
                                      Qt::AlignRight | Qt::AlignVCenter,
                                      values.elidedText(item.value, Qt::ElideRight, cellWidth - swatch));
                }
            }
            y += rowHeight;
        }
    }
    return y + 12;
}

void ViewportResultOverlay::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(borderColor(), 1));
    painter.setBrush(backgroundColor());
    painter.drawRect(QRectF(rect()).adjusted(.5, .5, -.5, -.5));
    layoutContents(&painter);
}
