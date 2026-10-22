#pragma once
#include "Measure/MeasureWeldParallelism.h"
#include <QObject>
#include <QColor>
#include <QPointer>
#include <vtkSmartPointer.h>
#include <memory>

class QWidget;
class QDockWidget;
class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;
class vtkRenderer;
class vtkActor;
class ViewportResultOverlay;
class WeldParallelismPlot;

class WeldParallelismTool : public QObject
{
    Q_OBJECT
public:
    struct Target {
        std::string name;
        MeasureWeldParallelism::Cloud::Ptr cloud;
        vtkSmartPointer<vtkActor> actor;
        QColor color=Qt::white;
    };
    struct Reference {
        std::string name;
        MeasureWeldParallelism::Cylinder cylinder;
    };
    struct Request {
        Target first,second;
        Reference reference;
        unsigned long ticket=0;
    };
    WeldParallelismTool(QWidget* view,vtkRenderer* renderer,ViewportResultOverlay* results);
    ~WeldParallelismTool() override;
    bool start(const std::vector<Target>& targets,const std::vector<Reference>& references,const std::string& preferred={});
    void stop();
    bool isActive() const { return active_; }
    bool isBusy() const { return busy_; }
    Request beginMeasurement();
    void setCancelling();
    bool finish(std::shared_ptr<MeasureWeldParallelism::Result> result,unsigned long ticket);
    void clearAnnotations();
signals:
    void activeChanged(bool active);
    void measureRequested();
    void cancelRequested();
    void resultMessage(const QString& text);
protected:
    bool eventFilter(QObject* watched,QEvent* event) override;
private:
    void buildPanel();
    void clearResult();
    void visualize();
    void updateUi();
    void updateVisibility();
    void render();
    QPointer<QWidget> view_;
    vtkRenderer* renderer_;
    ViewportResultOverlay* results_;
    QDockWidget* dock_=nullptr;
    QComboBox *first_=nullptr,*second_=nullptr,*reference_=nullptr;
    QPushButton *measure_=nullptr,*cancel_=nullptr,*clear_=nullptr;
    QLabel* summary_=nullptr;
    QCheckBox *points_=nullptr,*curves_=nullptr;
    WeldParallelismPlot* plot_=nullptr;
    std::vector<Target> targets_;
    std::vector<Reference> references_;
    std::shared_ptr<MeasureWeldParallelism::Result> result_;
    std::vector<vtkSmartPointer<vtkActor>> actors_;
    std::vector<std::pair<vtkSmartPointer<vtkActor>,int>> hidden_;
    unsigned long ticket_=0;
    bool active_=false,busy_=false,cancelling_=false;
};
