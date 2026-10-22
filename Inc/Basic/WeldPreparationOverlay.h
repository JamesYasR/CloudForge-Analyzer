#pragma once

#include <QWidget>
#include <QColor>
#include <vtkSmartPointer.h>
#include <array>
#include <vector>

class vtkRenderer;
class vtkCallbackCommand;
class ViewportResultOverlay;

class WeldPreparationOverlay : public QWidget
{
public:
    using Point = std::array<double, 3>;
    struct Annotation {
        Point anchor;
        std::vector<Point> endpoints;
        QString title;
        QString detail;
        QColor color;
    };

    WeldPreparationOverlay(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results = nullptr);
    ~WeldPreparationOverlay() override;
    void setAnnotations(std::vector<Annotation> annotations);
    void clear();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void paintEvent(QPaintEvent*) override;

private:
    bool project(const Point& point, QPointF& screen) const;
    vtkRenderer* renderer_ = nullptr;
    vtkSmartPointer<vtkCallbackCommand> callback_;
    unsigned long observer_ = 0;
    std::vector<Annotation> annotations_;
    ViewportResultOverlay* results_ = nullptr;
};
