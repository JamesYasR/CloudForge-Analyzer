#pragma once
#include "Measure/MeasureArc.h"
#include <QObject>
#include <QPointer>
#include <vector>
class QWidget;
class QDockWidget;
class QComboBox;
class QLineEdit;
class QFormLayout;
class QStackedWidget;
class QPushButton;
class ArcMeasurementTool : public QObject
{
    Q_OBJECT
public:
    explicit ArcMeasurementTool(QWidget* view);
    ~ArcMeasurementTool() override;
    bool start(bool geodesic,const std::vector<std::string>& clouds,const std::vector<std::string>& cylinders);
    void stop();
    bool isActive() const { return active_; }
    bool isGeodesic() const { return geodesic_; }
    bool isBusy() const { return busy_; }
    bool isPicking() const { return picking_; }
    void setBusy(bool busy,bool picking=false);
    bool usesPickedPosition() const;
    std::string cloudName() const;
    std::string cylinderName() const;
    bool parameters(MeasureArc::ArcParams& params,double& radius);
signals:
    void measureRequested();
    void cancelRequested();
    void clearRequested();
    void closed();
protected:
    bool eventFilter(QObject*,QEvent*) override;
private:
    void updateUi();
    QPointer<QWidget> view_;
    QDockWidget* dock_;
    QComboBox *cloud_,*reference_,*method_,*position_;
    QFormLayout* form_;
    QWidget* advanced_;
    QStackedWidget* methodFields_;
    std::vector<QLineEdit*> fields_;
    QPushButton *measure_,*cancel_,*clear_;
    bool active_=false,geodesic_=false,busy_=false,picking_=false,cancelling_=false;
};
