#pragma once

#include "Basic/CloudClipper.h"
#include <QObject>
#include <QPointF>
#include <QPointer>
#include <QCursor>
#include <QTimer>
#include <vtkSmartPointer.h>
#include <memory>

class QWidget;
class QFrame;
class QDockWidget;
class QLabel;
class QPushButton;
class QComboBox;
class QCheckBox;
class QPainter;
class vtkRenderer;
class vtkRenderWindow;
class vtkActor;

class PolygonClipTool : public QObject
{
    Q_OBJECT
public:
    struct Target {
        std::string name;
        CloudClipper::Cloud::Ptr cloud;
        vtkSmartPointer<vtkActor> actor;
    };
    enum class KeepMode { Inside, Outside, Both };

    PolygonClipTool(QWidget* view, vtkRenderer* renderer, vtkRenderWindow* window);
    ~PolygonClipTool() override;
    bool start(const std::vector<Target>& targets);
    void stop();
    bool isActive() const { return active_; }
    bool isBusy() const { return busy_; }
    CloudClipper::Request request() const;
    unsigned long ticket() const { return ticket_; }
    void finishPreview(const std::shared_ptr<CloudClipper::Result>& result, unsigned long ticket);
    bool canApply();
    void showMessage(const QString& message);
    const Target& target() const;
    KeepMode keepMode() const;
    std::shared_ptr<CloudClipper::Result> result() const { return result_; }

signals:
    void activeChanged(bool active);
    void previewRequested();
    void applyRequested();
    void cancelPreviewRequested();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void buildPanel();
    void paintContour(QPainter& painter);
    void closePolygon();
    void undoVertex();
    void resetPolygon();
    void rememberContour();
    void setDrawing(bool drawing);
    void preview();
    void clearPreview();
    void invalidatePreview();
    void updateUi();
    void updatePreviewColors();
    void render();
    int hitVertex(const QPointF& point) const;
    int hitEdge(const QPointF& point) const;
    QPointF normalized(const QPointF& point) const;
    QPointF displayed(const CloudClipper::Point2D& point) const;
    CloudClipper::Projection projection() const;
    vtkSmartPointer<vtkActor> previewActor(const CloudClipper::Cloud& cloud);

    QPointer<QWidget> view_;
    vtkRenderer* renderer_ = nullptr;
    vtkRenderWindow* window_ = nullptr;
    QWidget* canvas_ = nullptr;
    QFrame* panel_ = nullptr;
    QDockWidget* dock_ = nullptr;
    QWidget* body_ = nullptr;
    QComboBox* source_ = nullptr;
    QComboBox* mode_ = nullptr;
    QPushButton* draw_ = nullptr;
    QPushButton* navigate_ = nullptr;
    QPushButton* close_ = nullptr;
    QPushButton* undo_ = nullptr;
    QPushButton* reset_ = nullptr;
    QPushButton* preview_ = nullptr;
    QPushButton* apply_ = nullptr;
    QCheckBox* contourVisible_ = nullptr;
    QLabel* hint_ = nullptr;
    QLabel* count_ = nullptr;
    std::vector<Target> targets_;
    std::vector<CloudClipper::Point2D> polygon_;
    struct ContourState {
        std::vector<CloudClipper::Point2D> polygon;
        bool closed = false;
    };
    std::vector<ContourState> history_;
    std::shared_ptr<CloudClipper::Result> result_;
    vtkSmartPointer<vtkActor> insideActor_;
    vtkSmartPointer<vtkActor> outsideActor_;
    vtkSmartPointer<vtkActor> previewSource_;
    CloudClipper::Projection pendingProjection_;
    QTimer viewTimer_;
    QTimer renderTimer_;
    QPointF cursor_;
    QPointF press_;
    QCursor originalCursor_;
    unsigned long ticket_ = 0;
    int drag_ = -1;
    int selected_ = -1;
    bool active_ = false;
    bool drawing_ = true;
    bool closed_ = false;
    bool busy_ = false;
    bool pressed_ = false;
    bool moved_ = false;
    bool closeOnRelease_ = false;
    bool gestureSaved_ = false;
    bool mouseTracking_ = false;
    int originalVisibility_ = 1;
};
