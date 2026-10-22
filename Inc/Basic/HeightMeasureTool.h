#pragma once

#include "Measure/MeasureHeight.h"
#include <QObject>
#include <QPointer>
#include <vtkSmartPointer.h>
#include <memory>

class QWidget;
class QDockWidget;
class QComboBox;
class QSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;
class QTableWidget;
class vtkRenderer;
class vtkActor;
class ViewportResultOverlay;

class HeightMeasureTool : public QObject
{
    Q_OBJECT
public:
    struct Target {
        std::string name;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        vtkSmartPointer<vtkActor> actor;
    };
    struct Request {
        Target measured, reference;
        int neighbors = 10;
        int iterations = 5;
        unsigned long ticket = 0;
    };
    struct Result {
        std::shared_ptr<MeasureHeight> measurement;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colors;
        std::vector<double> averages;
        std::string message;
        bool success = false;
    };
    HeightMeasureTool(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results);
    ~HeightMeasureTool() override;
    bool start(const std::vector<Target>& targets);
    void stop();
    bool isActive() const { return active_; }
    bool isBusy() const { return busy_; }
    Request beginMeasurement();
    void setCancelling();
    bool finish(const std::shared_ptr<Result>& result, unsigned long ticket, bool cancelled);

signals:
    void activeChanged(bool active);
    void measureRequested();
    void cancelRequested();
    void resultMessage(const QString& text);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void buildPanel();
    void clearResult();
    void visualize(const Result& result);
    void updateVisibility();
    void updateUi();
    void render();
    QPointer<QWidget> view_;
    vtkRenderer* renderer_ = nullptr;
    ViewportResultOverlay* results_ = nullptr;
    QDockWidget* dock_ = nullptr;
    QComboBox* measured_ = nullptr;
    QComboBox* reference_ = nullptr;
    QSpinBox* neighbors_ = nullptr;
    QSpinBox* iterations_ = nullptr;
    QPushButton* measure_ = nullptr;
    QPushButton* cancel_ = nullptr;
    QPushButton* clear_ = nullptr;
    QLabel* summary_ = nullptr;
    QTableWidget* table_ = nullptr;
    QCheckBox* colors_ = nullptr;
    QCheckBox* plane_ = nullptr;
    QCheckBox* lines_ = nullptr;
    std::vector<Target> targets_;
    std::vector<std::pair<vtkSmartPointer<vtkActor>, int>> hidden_;
    std::vector<vtkSmartPointer<vtkActor>> actors_;
    vtkSmartPointer<vtkActor> heatmapActor_, referenceActor_, planeActor_, linesActor_;
    unsigned long ticket_ = 0;
    bool active_ = false;
    bool busy_ = false;
    bool cancelling_ = false;
};
