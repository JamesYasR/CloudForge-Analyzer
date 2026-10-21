#pragma once

#include "Measure/MeasureDistance.h"
#include <QObject>
#include <QPoint>
#include <QPointer>
#include <QCursor>
#include <QTimer>
#include <vtkSmartPointer.h>
#include <vector>

class QWidget;
class QLabel;
class vtkActor;
class vtkRenderer;
class vtkRenderWindow;
class vtkPointPicker;
class vtkSphereSource;
class vtkLineSource;
class vtkBillboardTextActor3D;
class vtkCallbackCommand;

class DistanceMeasureTool : public QObject
{
    Q_OBJECT
public:
    DistanceMeasureTool(QWidget* view, vtkRenderer* renderer, vtkRenderWindow* window);
    ~DistanceMeasureTool() override;

    bool start(const std::vector<vtkActor*>& clouds);
    void stop();
    void clear();
    bool isActive() const { return active_; }

signals:
    void activeChanged(bool active);
    void resultChanged(const QString& text);
    void measurementCompleted(const QString& text);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    enum class Stage { Start, End, Complete };
    bool pick(const QPoint& pos, MeasureDistance::Point& point);
    void hover();
    void select(const QPoint& pos);
    void showEnd(const MeasureDistance::Point& point);
    void hidePreview();
    void updateAnnotations();
    void render();
    void setHint(const QString& text);
    QString resultText() const;

    QPointer<QWidget> view_;
    QLabel* hint_ = nullptr;
    vtkRenderer* renderer_ = nullptr;
    vtkSmartPointer<vtkRenderer> overlayRenderer_;
    vtkRenderWindow* window_ = nullptr;
    vtkSmartPointer<vtkPointPicker> picker_;
    vtkSmartPointer<vtkLineSource> line_;
    vtkSmartPointer<vtkActor> lineActor_;
    vtkSmartPointer<vtkActor> lineOutline_;
    vtkSmartPointer<vtkSphereSource> spheres_[2];
    vtkSmartPointer<vtkActor> points_[2];
    vtkSmartPointer<vtkBillboardTextActor3D> labels_[3];
    vtkSmartPointer<vtkCallbackCommand> renderCallback_;
    unsigned long renderObserver_ = 0;
    QTimer hoverTimer_;
    QPoint cursorPos_;
    QPoint pressPos_;
    Stage stage_ = Stage::Start;
    MeasureDistance::Result result_;
    double textHeight_ = 6.0;
    bool active_ = false;
    bool pressed_ = false;
    bool dragged_ = false;
    bool mouseTracking_ = false;
    QCursor originalCursor_;
};
