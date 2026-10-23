#pragma once

#include <QWidget>
#include <QColor>
#include <QPointer>
#include <vtkSmartPointer.h>
#include <array>
#include <vector>

class QComboBox;
class QLabel;
class QFont;
class vtkRenderer;
class vtkCallbackCommand;
class ViewportResultOverlay;

class ViewportAnnotationOverlay : public QWidget
{
public:
    using Point = std::array<double, 3>;
    struct Annotation {
        Point anchor;
        std::vector<Point> endpoints;
        QString title;
        QString detail;
        QColor color;
        QString marker;
        bool primary = false;
    };

    ViewportAnnotationOverlay(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results = nullptr, bool filledMarkers = false);
    ~ViewportAnnotationOverlay() override;
    void setAnnotations(std::vector<Annotation> annotations);
    void clear();
    void avoidWidget(QWidget* widget);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void paintEvent(QPaintEvent*) override;

private:
    bool project(const Point& point, QPointF& screen) const;
    bool layoutCards(const std::vector<size_t>& indices, const QFont& title, const QFont& detail,
                     bool compact, bool selector, std::vector<QRectF>& cards) const;
    vtkRenderer* renderer_ = nullptr;
    vtkSmartPointer<vtkCallbackCommand> callback_;
    unsigned long observer_ = 0;
    std::vector<Annotation> annotations_;
    ViewportResultOverlay* results_ = nullptr;
    QPointer<QWidget> obstacle_;
    bool filledMarkers_ = false;
    QWidget* selectorBar_ = nullptr;
    QComboBox* selector_ = nullptr;
    QLabel* selectedDetails_ = nullptr;
    bool selective_ = false;
    std::vector<QRectF> cards_;
};
