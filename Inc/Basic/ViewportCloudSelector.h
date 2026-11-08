#pragma once

#include <QObject>
#include <QPointer>
#include <QPoint>
#include <QCursor>
#include <vtkSmartPointer.h>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

class QWidget;
class QDialog;
class QAction;
class vtkActor;
class vtkRenderer;
class vtkRenderWindow;
class vtkPointPicker;

// GUI 线程内共享的视图选择服务；选择窗口只传点云标识，不管理查看器或场景数据。
class ViewportCloudSelector : public QObject {
    Q_OBJECT
public:
    using Actors = std::map<std::string, vtkSmartPointer<vtkActor>>;
    using ReadActors = std::function<Actors()>;
    ViewportCloudSelector(QWidget* view, vtkRenderer* renderer, vtkRenderWindow* window,
                          ReadActors read, QObject* parent);
    ~ViewportCloudSelector() override;
    bool begin(QDialog* dialog, const std::set<std::string>& eligible);
    void end(QDialog* dialog);
    void setPicking(bool enabled);
    void setSelected(const std::set<std::string>& selected);

signals:
    void selectionStarted();
    void cloudClicked(const QString& id);
    void pickingChanged(bool enabled);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void updateHighlights();
    void pick(const QPoint& position);
    QPointer<QWidget> view_;
    QPointer<QDialog> dialog_;
    vtkRenderer* renderer_;
    vtkRenderWindow* window_;
    ReadActors read_;
    vtkSmartPointer<vtkPointPicker> picker_;
    Actors actors_;
    std::set<std::string> selected_;
    std::vector<vtkSmartPointer<vtkActor>> highlights_;
    std::vector<std::pair<QPointer<QAction>, bool>> actions_;
    QCursor cursor_;
    QPoint press_;
    bool picking_ = false;
    bool pressed_ = false;
    bool dragged_ = false;
};
