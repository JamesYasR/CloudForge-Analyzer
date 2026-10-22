#include "Basic/HeightMeasureTool.h"
#include "Basic/ViewportResultOverlay.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QEvent>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkGlyph3DMapper.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
vtkSmartPointer<vtkActor> actorFor(vtkPolyData* data)
{
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(data);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->PickableOff();
    actor->GetProperty()->LightingOff();
    return actor;
}
vtkSmartPointer<vtkPolyData> pointData(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto vertices = vtkSmartPointer<vtkCellArray>::New();
    for (const auto& p : cloud) {
        vtkIdType id = points->InsertNextPoint(p.x, p.y, p.z);
        vertices->InsertNextCell(1, &id);
    }
    auto data = vtkSmartPointer<vtkPolyData>::New();
    data->SetPoints(points); data->SetVerts(vertices);
    return data;
}
}

HeightMeasureTool::HeightMeasureTool(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results)
    : view_(view), renderer_(renderer), results_(results)
{
    buildPanel();
    qApp->installEventFilter(this);
}

HeightMeasureTool::~HeightMeasureTool()
{
    stop();
    qApp->removeEventFilter(this);
    delete dock_;
}

void HeightMeasureTool::buildPanel()
{
    dock_ = new QDockWidget("高度测量", view_->window());
    dock_->setObjectName("heightMeasureDock");
    dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    auto body = new QWidget(dock_);
    body->setMinimumWidth(290);
    body->setStyleSheet("QPushButton { border: 1px solid palette(mid); padding: 6px 8px; border-radius: 0; }"
                       "QPushButton:hover { background: palette(light); } QPushButton:disabled { color: palette(mid); }");
    auto layout = new QVBoxLayout(body);
    auto form = new QFormLayout();
    measured_ = new QComboBox(body); measured_->setObjectName("heightMeasured");
    reference_ = new QComboBox(body); reference_->setObjectName("heightReference");
    form->addRow("测量点云", measured_); form->addRow("参考面点云", reference_);
    layout->addLayout(form);
    auto settings = new QGroupBox("测量设置", body);
    auto parameters = new QFormLayout(settings);
    neighbors_ = new QSpinBox(settings); neighbors_->setObjectName("heightNeighbors");
    neighbors_->setRange(1, 100000); neighbors_->setValue(10);
    iterations_ = new QSpinBox(settings); iterations_->setObjectName("heightIterations");
    iterations_->setRange(1, 10000); iterations_->setValue(5);
    parameters->addRow("邻近点数量", neighbors_); parameters->addRow("测量次数", iterations_);
    layout->addWidget(settings);
    measure_ = new QPushButton("开始测量", body); measure_->setObjectName("heightMeasure");
    measure_->setToolTip("开始测量（Ctrl+Enter）");
    cancel_ = new QPushButton("取消计算", body); cancel_->setObjectName("heightCancel");
    auto buttons = new QHBoxLayout(); buttons->addWidget(measure_); buttons->addWidget(cancel_);
    layout->addLayout(buttons);
    summary_ = new QLabel(body); summary_->setObjectName("heightSummary"); summary_->setWordWrap(true);
    layout->addWidget(summary_);
    table_ = new QTableWidget(0, 2, body); table_->setObjectName("heightRegions");
    table_->setHorizontalHeaderLabels({"测量序号", "平均高度 (mm)"});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->verticalHeader()->hide();
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setMaximumHeight(180);
    table_->setMinimumHeight(95);
    layout->addWidget(table_);
    auto display = new QGroupBox("显示", body);
    auto displayLayout = new QVBoxLayout(display);
    colors_ = new QCheckBox("高度着色", display); colors_->setObjectName("heightColors"); colors_->setChecked(true);
    plane_ = new QCheckBox("参考平面", display); plane_->setObjectName("heightPlane"); plane_->setChecked(true);
    lines_ = new QCheckBox("辅助高度线", display); lines_->setObjectName("heightLines"); lines_->setChecked(true);
    for (auto box : {colors_, plane_, lines_}) {
        displayLayout->addWidget(box);
        connect(box, &QCheckBox::toggled, this, &HeightMeasureTool::updateVisibility);
    }
    layout->addWidget(display);
    clear_ = new QPushButton("清除结果", body); clear_->setObjectName("heightClear");
    auto exit = new QPushButton("结束测量", body); exit->setObjectName("heightExit"); exit->setToolTip("结束测量（Esc / Q）");
    layout->addWidget(clear_); layout->addWidget(exit); layout->addStretch();
    dock_->setWidget(body);
    if (auto main = qobject_cast<QMainWindow*>(view_->window())) {
        main->addDockWidget(Qt::LeftDockWidgetArea, dock_);
        main->resizeDocks({dock_}, {320}, Qt::Horizontal);
    }
    dock_->hide();
    connect(measure_, &QPushButton::clicked, this, &HeightMeasureTool::measureRequested);
    connect(cancel_, &QPushButton::clicked, this, [this]() { setCancelling(); emit cancelRequested(); });
    connect(exit, &QPushButton::clicked, this, &HeightMeasureTool::stop);
    connect(clear_, &QPushButton::clicked, this, [this]() { clearResult(); });
    for (auto box : {measured_, reference_}) connect(box, &QComboBox::currentIndexChanged, this, [this]() {
        if (active_ && !busy_) { ++ticket_; clearResult(); }
    });
    for (auto spin : {neighbors_, iterations_}) connect(spin, &QSpinBox::valueChanged, this, [this]() {
        if (active_ && !busy_) { ++ticket_; clearResult(); }
    });
    dock_->installEventFilter(this);
    updateUi();
}

bool HeightMeasureTool::start(const std::vector<Target>& targets)
{
    stop();
    targets_.clear();
    for (const auto& target : targets) if (target.actor && target.cloud && !target.cloud->empty()) targets_.push_back(target);
    if (targets_.empty()) return false;
    {
        QSignalBlocker first(measured_), second(reference_);
        measured_->clear(); reference_->clear();
        for (const auto& target : targets_) {
            measured_->addItem(QString::fromStdString(target.name)); reference_->addItem(QString::fromStdString(target.name));
        }
        reference_->setCurrentIndex(targets_.size() > 1 ? 1 : 0);
    }
    active_ = true;

    dock_->show(); dock_->raise();
    updateUi();
    emit activeChanged(true);
    return true;
}

void HeightMeasureTool::stop()
{
    if (!active_ && !busy_ && actors_.empty() && hidden_.empty()) return;
    const bool wasActive = active_, wasBusy = busy_;
    active_ = busy_ = false;
    ++ticket_;
    if (wasBusy) emit cancelRequested();
    clearResult();
    dock_->hide();
    targets_.clear();
    if (wasActive) emit activeChanged(false);
}

HeightMeasureTool::Request HeightMeasureTool::beginMeasurement()
{
    if (!active_ || busy_) return {};
    clearResult();
    busy_ = true; cancelling_ = false;
    updateUi();
    return {targets_.at(measured_->currentIndex()), targets_.at(reference_->currentIndex()),
            neighbors_->value(), iterations_->value(), ++ticket_};
}

void HeightMeasureTool::setCancelling()
{
    if (!busy_) return;
    cancelling_ = true;
    updateUi();
}

bool HeightMeasureTool::finish(const std::shared_ptr<Result>& result, unsigned long ticket, bool cancelled)
{
    if (!active_ || !busy_ || ticket != ticket_) return false;
    busy_ = false;
    if (cancelled || cancelling_) { emit resultMessage("高度测量已取消"); updateUi(); return false; }
    if (!result || !result->success) {
        emit resultMessage(result && !result->message.empty() ? QString::fromStdString(result->message) : "不可测: 未能获得高度结果");
        updateUi(); return false;
    }
    visualize(*result);
    const auto& measurement = *result->measurement;
    summary_->setText(QString("最大 %1 mm\n最小 %2 mm\n平均 %3 mm")
        .arg(measurement.GetMaxDistance(), 0, 'f', 3).arg(measurement.GetMinDistance(), 0, 'f', 3).arg(measurement.GetMeanDistance(), 0, 'f', 3));
    table_->setRowCount(static_cast<int>(result->averages.size()));
    for (int i = 0; i < table_->rowCount(); ++i) {
        table_->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        table_->setItem(i, 1, new QTableWidgetItem(QString::number(result->averages[i], 'f', 4)));
    }
    results_->setSection("height", {"高度测量", {
        {"最大高度", QString::number(measurement.GetMaxDistance(), 'f', 3) + " mm", QColor("#ffda55")},
        {"最小高度", QString::number(measurement.GetMinDistance(), 'f', 3) + " mm", {}},
        {"平均高度", QString::number(measurement.GetMeanDistance(), 'f', 3) + " mm", {}}
    }});

    updateUi(); updateVisibility();
    return true;
}

void HeightMeasureTool::clearResult()
{
    for (auto& actor : actors_) renderer_->RemoveActor(actor);
    actors_.clear();
    heatmapActor_ = referenceActor_ = planeActor_ = linesActor_ = nullptr;
    for (auto& hidden : hidden_) hidden.first->SetVisibility(hidden.second);
    hidden_.clear();
    results_->removeSection("height");
    table_->setRowCount(0); summary_->clear();
    updateUi(); render();
}

void HeightMeasureTool::visualize(const Result& result)
{
    const auto& source = targets_.at(measured_->currentIndex());
    const auto& reference = targets_.at(reference_->currentIndex());
    for (auto actor : {source.actor, reference.actor}) {
        if (std::none_of(hidden_.begin(), hidden_.end(), [&](const auto& item) { return item.first == actor; })) {
            hidden_.push_back({actor, actor->GetVisibility()}); actor->VisibilityOff();
        }
    }
    auto add = [&](vtkSmartPointer<vtkActor> actor) {
        renderer_->AddActor(actor); actors_.push_back(actor); return actor;
    };
    auto data = pointData(*source.cloud);
    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New(); colors->SetNumberOfComponents(3);
    for (const auto& point : *result.colors) { const unsigned char rgb[3] = {point.r, point.g, point.b}; colors->InsertNextTypedTuple(rgb); }
    data->GetPointData()->SetScalars(colors);
    heatmapActor_ = add(actorFor(data)); heatmapActor_->GetProperty()->SetPointSize(3);
    referenceActor_ = add(actorFor(pointData(*reference.cloud)));
    referenceActor_->GetProperty()->SetColor(.25, .65, 1); referenceActor_->GetProperty()->SetPointSize(2);
    const auto coeff = result.measurement->GetPlaneCoefficients();
    Eigen::Vector3d normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    const double normSquared = normal.squaredNorm(), d = coeff->values[3];
    const Eigen::Vector3d u = normal.unitOrthogonal(), v = normal.normalized().cross(u);
    double uMin = std::numeric_limits<double>::max(), uMax = -uMin, vMin = uMin, vMax = -uMin;
    for (const auto& p : *source.cloud) {
        Eigen::Vector3d point(p.x, p.y, p.z);
        uMin = std::min(uMin, point.dot(u)); uMax = std::max(uMax, point.dot(u));
        vMin = std::min(vMin, point.dot(v)); vMax = std::max(vMax, point.dot(v));
    }
    const double margin = std::max({uMax - uMin, vMax - vMin, .01}) * .06;
    const auto base = -d * normal / normSquared;
    const Eigen::Vector3d origin = base + (uMin-margin)*u + (vMin-margin)*v;
    const Eigen::Vector3d first = base + (uMax+margin)*u + (vMin-margin)*v;
    const Eigen::Vector3d second = base + (uMin-margin)*u + (vMax+margin)*v;
    auto plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(origin.data());
    plane->SetPoint1(first[0], first[1], first[2]); plane->SetPoint2(second[0], second[1], second[2]); plane->Update();
    planeActor_ = add(actorFor(plane->GetOutput())); planeActor_->UseBoundsOff();
    planeActor_->GetProperty()->SetColor(.25, .65, 1); planeActor_->GetProperty()->SetOpacity(.16);
    auto linePoints = vtkSmartPointer<vtkPoints>::New(); auto cells = vtkSmartPointer<vtkCellArray>::New();
    double maxDistance = -1; Eigen::Vector3d maximum = Eigen::Vector3d::Zero(), foot = maximum;
    const size_t stride = std::max<size_t>(1, source.cloud->size() / 50);
    for (size_t i = 0; i < source.cloud->size(); ++i) {
        const auto& p = (*source.cloud)[i]; const Eigen::Vector3d point(p.x,p.y,p.z);
        const double signedDistance = normal.dot(point) + d;
        const Eigen::Vector3d projection = point - signedDistance * normal / normSquared;
        if (std::abs(signedDistance) > maxDistance) { maxDistance = std::abs(signedDistance); maximum = point; foot = projection; }
        if (i % stride) continue;
        const vtkIdType ids[2] = {linePoints->InsertNextPoint(point.data()), linePoints->InsertNextPoint(projection.data())};
        cells->InsertNextCell(2, ids);
    }
    auto lines = vtkSmartPointer<vtkPolyData>::New(); lines->SetPoints(linePoints); lines->SetLines(cells);
    linesActor_ = add(actorFor(lines)); linesActor_->UseBoundsOff();
    linesActor_->GetProperty()->SetColor(.5,.65,1); linesActor_->GetProperty()->SetOpacity(.4);
    auto maxPoints = vtkSmartPointer<vtkPoints>::New(); maxPoints->InsertNextPoint(maximum.data()); maxPoints->InsertNextPoint(foot.data());
    auto maxCells = vtkSmartPointer<vtkCellArray>::New(); const vtkIdType ids[2] = {0,1}; maxCells->InsertNextCell(2,ids);
    auto maxData = vtkSmartPointer<vtkPolyData>::New(); maxData->SetPoints(maxPoints); maxData->SetLines(maxCells);
    auto maximumActor = add(actorFor(maxData)); maximumActor->UseBoundsOff();
    maximumActor->GetProperty()->SetColor(1,.85,.2); maximumActor->GetProperty()->SetLineWidth(4);
    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(std::max(.003 * std::hypot(uMax-uMin,vMax-vMin), .001));
    auto markers = vtkSmartPointer<vtkGlyph3DMapper>::New(); markers->SetInputData(maxData); markers->SetSourceConnection(sphere->GetOutputPort()); markers->ScalingOff();
    auto ends = vtkSmartPointer<vtkActor>::New(); ends->SetMapper(markers); ends->PickableOff(); ends->UseBoundsOff();
    ends->GetProperty()->SetColor(1,.85,.2); ends->GetProperty()->LightingOff(); add(ends);
}

void HeightMeasureTool::updateVisibility()
{
    if (heatmapActor_) {
        heatmapActor_->SetVisibility(colors_->isChecked());
        targets_.at(measured_->currentIndex()).actor->SetVisibility(!colors_->isChecked());
    }
    if (referenceActor_) referenceActor_->SetVisibility(plane_->isChecked()
        && targets_.at(measured_->currentIndex()).actor != targets_.at(reference_->currentIndex()).actor);
    if (planeActor_) planeActor_->SetVisibility(plane_->isChecked());
    if (linesActor_) linesActor_->SetVisibility(lines_->isChecked());
    render();
}

void HeightMeasureTool::updateUi()
{
    for (auto box : {measured_, reference_}) box->setEnabled(!busy_);
    neighbors_->setEnabled(!busy_); iterations_->setEnabled(!busy_);
    measure_->setEnabled(active_ && !busy_ && !targets_.empty());
    measure_->setText(busy_ ? "正在测量" : "开始测量");
    cancel_->setText(cancelling_ && busy_ ? "正在取消" : "取消计算");
    cancel_->setEnabled(busy_ && !cancelling_);
    clear_->setEnabled(!busy_ && !actors_.empty());
}

bool HeightMeasureTool::eventFilter(QObject* watched, QEvent* event)
{
    if (!active_) return false;
    if (event->type() == QEvent::Close && (watched == dock_ || watched == view_->window())) stop();
    if (event->type() == QEvent::KeyPress) {
        auto widget = qobject_cast<QWidget*>(watched);
        if (widget && (widget->window() == view_->window() || widget->window() == dock_->window())) {
            const auto key = static_cast<QKeyEvent*>(event);
            if (key->key() == Qt::Key_Escape || (watched == view_ && key->key() == Qt::Key_Q && key->modifiers() == Qt::NoModifier)) {
                stop(); return true;
            }
            if ((key->key() == Qt::Key_Return || key->key() == Qt::Key_Enter) && key->modifiers() == Qt::ControlModifier && !busy_) {
                emit measureRequested(); return true;
            }
        }
    }
    return false;
}

void HeightMeasureTool::render()
{
    if (view_ && view_->isVisible()) { renderer_->GetRenderWindow()->Render(); view_->update(); }
}
