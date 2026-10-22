#include "Basic/ViewportPointPicker.h"
#include <QApplication>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QWidget>
#include <vtkActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkLineSource.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <algorithm>
#include <cmath>
#include <limits>

ViewportPointPicker::ViewportPointPicker(QWidget* view, vtkRenderer* renderer, vtkRenderWindow* window)
    : view_(view), renderer_(renderer), window_(window)
{
    overlayRenderer_ = vtkSmartPointer<vtkRenderer>::New();
    overlayRenderer_->SetLayer(window_->GetNumberOfLayers());
    window_->SetNumberOfLayers(window_->GetNumberOfLayers() + 1);
    overlayRenderer_->InteractiveOff();
    overlayRenderer_->PreserveColorBufferOn();
    overlayRenderer_->PreserveDepthBufferOff();
    overlayRenderer_->DrawOff();
    window_->AddRenderer(overlayRenderer_);
    hint_ = new QLabel(view);
    hint_->setObjectName("distanceMeasureHint");
    hint_->setAttribute(Qt::WA_TransparentForMouseEvents);
    hint_->setStyleSheet("QLabel { color: #fff2ac; background: rgba(15, 27, 35, 220);"
                         " padding: 9px 12px; border-radius: 2px; font-size: 14px; }");
    hint_->move(12, 12);
    hint_->hide();
    cancel_ = new QPushButton("取消", view);
    cancel_->setObjectName("pointPickCancel");
    cancel_->setStyleSheet("QPushButton { color: white; background: #183b49; border: 1px solid #62808b;"
                          " padding: 7px 12px; border-radius: 2px; } QPushButton:hover { background: #2c5261; }");
    cancel_->hide();
    connect(cancel_, &QPushButton::clicked, this, &ViewportPointPicker::stop);
    picker_ = vtkSmartPointer<vtkPointPicker>::New();
    picker_->PickFromListOn();
    picker_->UseCellsOff();
    line_ = vtkSmartPointer<vtkLineSource>::New();
    auto lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    lineMapper->SetInputConnection(line_->GetOutputPort());
    lineOutline_ = vtkSmartPointer<vtkActor>::New();
    lineActor_ = vtkSmartPointer<vtkActor>::New();
    for (auto actor : {lineOutline_, lineActor_}) {
        actor->SetMapper(lineMapper);
        actor->PickableOff();
        actor->UseBoundsOff();
        actor->GetProperty()->LightingOff();
        actor->VisibilityOff();
        overlayRenderer_->AddActor(actor);
    }
    lineOutline_->GetProperty()->SetColor(0.03, 0.06, 0.08);
    lineOutline_->GetProperty()->SetLineWidth(8.0);
    lineActor_->GetProperty()->SetColor(1.0, 0.85, 0.2);
    lineActor_->GetProperty()->SetLineWidth(3.0);
    for (int i = 0; i < 2; ++i) {
        spheres_[i] = vtkSmartPointer<vtkSphereSource>::New();
        spheres_[i]->SetThetaResolution(16);
        spheres_[i]->SetPhiResolution(16);
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(spheres_[i]->GetOutputPort());
        points_[i] = vtkSmartPointer<vtkActor>::New();
        points_[i]->SetMapper(mapper);
        points_[i]->GetProperty()->SetColor(i == 0 ? 0.2 : 1.0, i == 0 ? 0.95 : 0.65, i == 0 ? 1.0 : 0.2);
        points_[i]->GetProperty()->LightingOff();
        points_[i]->PickableOff();
        points_[i]->UseBoundsOff();
        points_[i]->VisibilityOff();
        overlayRenderer_->AddActor(points_[i]);
    }
    for (int i = 0; i < 3; ++i) {
        labels_[i] = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        labels_[i]->PickableOff();
        labels_[i]->UseBoundsOff();
        labels_[i]->VisibilityOff();
        auto property = labels_[i]->GetTextProperty();
        property->SetColor(1.0, 0.94, 0.7);
        property->SetBackgroundColor(0.03, 0.06, 0.08);
        property->SetBackgroundOpacity(0.85);
        property->BoldOn();
        labels_[i]->SetDisplayOffset(12, 12);
        overlayRenderer_->AddActor(labels_[i]);
    }
    labels_[0]->SetInput("A");
    labels_[1]->SetInput("B");
    labels_[2]->GetTextProperty()->SetJustificationToCentered();
    renderCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    renderCallback_->SetClientData(this);
    renderCallback_->SetCallback([](vtkObject*, unsigned long, void* data, void*) {
        static_cast<ViewportPointPicker*>(data)->updateAnnotations();
    });
    renderObserver_ = renderer_->AddObserver(vtkCommand::StartEvent, renderCallback_);
    hoverTimer_.setSingleShot(true);
    hoverTimer_.setInterval(25);
    connect(&hoverTimer_, &QTimer::timeout, this, &ViewportPointPicker::hover);
    qApp->installEventFilter(this);
}

ViewportPointPicker::~ViewportPointPicker()
{
    stop();
    renderer_->RemoveObserver(renderObserver_);
    window_->RemoveRenderer(overlayRenderer_);
    qApp->removeEventFilter(this);
    delete hint_;
    delete cancel_;
}

bool ViewportPointPicker::start(const std::vector<vtkActor*>& clouds)
{
    if (active_) stop();
    selecting_ = false;
    selectedPoints_.clear();
    return begin(clouds);
}

bool ViewportPointPicker::begin(const std::vector<vtkActor*>& clouds)
{
    picker_->InitializePickList();
    double bounds[6] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
    bool available = false;
    for (auto actor : clouds) {
        if (!actor || !actor->GetVisibility() || actor->GetProperty()->GetOpacity() <= 0.0) continue;
        double* b = actor->GetBounds();
        if (!b || b[0] > b[1]) continue;
        picker_->AddPickList(actor);
        available = true;
        for (int i = 0; i < 3; ++i) {
            bounds[2 * i] = std::min(bounds[2 * i], b[2 * i]);
            bounds[2 * i + 1] = std::max(bounds[2 * i + 1], b[2 * i + 1]);
        }
    }
    if (!available || !view_) return false;
    textHeight_ = std::clamp(0.022 * std::hypot(bounds[1] - bounds[0], bounds[3] - bounds[2],
                                              bounds[5] - bounds[4]), 6.0, 14.0);
    mouseTracking_ = view_->hasMouseTracking();
    originalCursor_ = view_->cursor();
    view_->setMouseTracking(true);
    view_->setCursor(Qt::CrossCursor);
    view_->setFocus(Qt::OtherFocusReason);
    active_ = true;
    stage_ = result_.success ? Stage::Complete : Stage::Start;
    pressed_ = dragged_ = false;
    if (selecting_) setHint(selectionTitle_ + (targetPoints_ == 1 ? " · 选择位置 · Esc 取消" : " · 选择起点 · Esc 取消"));
    else setHint(stage_ == Stage::Complete ? "选择起点继续测量 · Esc 退出" : "选择起点 · Esc 退出");
    emit activeChanged(true);
    return true;
}

bool ViewportPointPicker::startSelection(const std::vector<vtkActor*>& clouds, int count, const QString& title)
{
    clear();
    if (count < 1 || count > 2) return false;
    selecting_ = true;
    targetPoints_ = count;
    selectionTitle_ = title;
    selectedPoints_.clear();
    if (!begin(clouds)) { selecting_ = false; return false; }
    return true;
}

void ViewportPointPicker::stop()
{
    if (!active_) return;
    const bool selection = selecting_;
    const bool accepted = selection && selectedPoints_.size() == static_cast<size_t>(targetPoints_);
    hoverTimer_.stop();
    if (selection || stage_ != Stage::Complete) {
        result_ = {};
        for (auto& point : points_) point->VisibilityOff();
        for (auto& label : labels_) label->VisibilityOff();
        lineActor_->VisibilityOff();
        lineOutline_->VisibilityOff();
        if (!selection) emit resultChanged(QString());
    }
    active_ = false;
    pressed_ = dragged_ = false;
    picker_->InitializePickList();
    if (view_) {
        view_->setMouseTracking(mouseTracking_);
        view_->setCursor(originalCursor_);
    }
    hint_->hide();
    cancel_->hide();
    emit activeChanged(false);
    render();
    if (selection) {
        if (!accepted) selectedPoints_.clear();
        selecting_ = false;
        emit selectionCompleted(accepted);
    }
}

void ViewportPointPicker::clear()
{
    selectedPoints_.clear();
    const bool hadMeasurement = points_[0]->GetVisibility() || result_.success;
    if (!active_ && !hadMeasurement) return;
    stop();
    result_ = {};
    stage_ = Stage::Start;
    for (auto& point : points_) point->VisibilityOff();
    for (auto& label : labels_) label->VisibilityOff();
    lineActor_->VisibilityOff();
    lineOutline_->VisibilityOff();
    if (hadMeasurement) emit resultChanged(QString());
    render();
}

bool ViewportPointPicker::eventFilter(QObject* watched, QEvent* event)
{
    if (!active_ || !view_) return false;
    if (watched == view_->window() && event->type() == QEvent::Close) stop();
    if (event->type() == QEvent::KeyPress) {
        auto widget = qobject_cast<QWidget*>(watched);
        if (widget && widget->window() == view_->window() && static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
            stop();
            return true;
        }
    }
    if (watched != view_) return false;
    if (event->type() == QEvent::Resize) setHint(hint_->text());
    if (event->type() == QEvent::MouseButtonPress) {
        auto mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton && mouse->modifiers() == Qt::NoModifier) {
            pressPos_ = mouse->position().toPoint();
            pressed_ = true;
            dragged_ = false;
        }
        hoverTimer_.stop();
        if (stage_ == Stage::End) hidePreview();
    }
    else if (event->type() == QEvent::MouseMove) {
        auto mouse = static_cast<QMouseEvent*>(event);
        cursorPos_ = mouse->position().toPoint();
        if (pressed_ && (cursorPos_ - pressPos_).manhattanLength() >= QApplication::startDragDistance()) dragged_ = true;
        if (mouse->buttons() == Qt::NoButton && stage_ != Stage::Complete && !hoverTimer_.isActive()) hoverTimer_.start();
    }
    else if (event->type() == QEvent::MouseButtonRelease) {
        auto mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
            const bool click = pressed_ && !dragged_ && mouse->modifiers() == Qt::NoModifier
                && (mouse->position().toPoint() - pressPos_).manhattanLength() < QApplication::startDragDistance();
            pressed_ = false;
            if (click) select(mouse->position().toPoint());
        }
    }
    else if (event->type() == QEvent::Leave || event->type() == QEvent::Wheel) {
        hoverTimer_.stop();
        if (stage_ != Stage::Complete) hidePreview();
    }
    return false;
}

bool ViewportPointPicker::pick(const QPoint& pos, MeasureDistance::Point& point)
{
    if (!view_ || !view_->rect().contains(pos)) return false;
    const int* size = window_->GetSize();
    if (size[0] <= 0 || size[1] <= 0 || view_->width() <= 0 || view_->height() <= 0) return false;
    const double scale = double(size[0]) / view_->width();
    picker_->SetTolerance(6.0 * scale / std::hypot(size[0], size[1]));
    if (!picker_->Pick(pos.x() * scale, size[1] - 1 - pos.y() * double(size[1]) / view_->height(), 0.0, renderer_)
        || picker_->GetPointId() < 0) return false;
    const double* p = picker_->GetPickPosition();
    point = {p[0], p[1], p[2]};
    return MeasureDistance::compute(point, point).success;
}

void ViewportPointPicker::hover()
{
    if (!active_ || pressed_ || stage_ == Stage::Complete) return;
    MeasureDistance::Point point;
    if (!pick(cursorPos_, point)) {
        hidePreview();
        return;
    }
    if (stage_ == Stage::Start) {
        spheres_[0]->SetCenter(point.data());
        points_[0]->VisibilityOn();
    }
    else showEnd(point);
    render();
}

void ViewportPointPicker::select(const QPoint& pos)
{
    hoverTimer_.stop();
    MeasureDistance::Point point;
    if (!pick(pos, point)) {
        if (stage_ != Stage::Complete) hidePreview();
        return;
    }
    if (selecting_) {
        selectedPoints_.push_back(point);
        if (selectedPoints_.size() == 1) {
            result_.start = point;
            stage_ = Stage::End;
            spheres_[0]->SetCenter(point.data());
            points_[0]->VisibilityOn();
            labels_[0]->SetPosition(point.data());
            labels_[0]->VisibilityOn();
            setHint(selectionTitle_ + " · 选择终点 · 拖动旋转 / 滚轮缩放 · Esc 取消");
        }
        if (selectedPoints_.size() == static_cast<size_t>(targetPoints_)) stop();
        else render();
        return;
    }
    if (stage_ != Stage::End) {
        result_ = {};
        result_.start = point;
        stage_ = Stage::End;
        hidePreview();
        spheres_[0]->SetCenter(point.data());
        points_[0]->VisibilityOn();
        labels_[0]->SetPosition(point.data());
        labels_[0]->VisibilityOn();
        setHint("选择终点 · 拖动旋转 / 滚轮缩放 · Esc 退出");
    }
    else {
        showEnd(point);
        if (result_.success) {
            stage_ = Stage::Complete;
            setHint("选择起点继续测量 · Esc 退出");
            emit measurementCompleted(resultText());
        }
    }
    render();
}

void ViewportPointPicker::showEnd(const MeasureDistance::Point& point)
{
    result_ = MeasureDistance::compute(result_.start, point);
    if (!result_.success) { hidePreview(); return; }
    spheres_[1]->SetCenter(point.data());
    points_[1]->VisibilityOn();
    labels_[1]->SetPosition(point[0], point[1], point[2]);
    labels_[1]->VisibilityOn();
    if (selecting_) return;
    line_->SetPoint1(result_.start.data());
    line_->SetPoint2(point.data());
    lineActor_->VisibilityOn();
    lineOutline_->VisibilityOn();
    MeasureDistance::Point middle;
    for (int i = 0; i < 3; ++i) middle[i] = result_.start[i] * 0.5 + point[i] * 0.5;
    labels_[2]->SetPosition(middle.data());
    const auto text = QString::number(result_.distance, 'f', 3) + " mm";
    labels_[2]->SetInput(text.toUtf8().constData());
    labels_[2]->VisibilityOn();
    emit resultChanged(resultText());
}

void ViewportPointPicker::hidePreview()
{
    if (stage_ == Stage::Start) points_[0]->VisibilityOff();
    points_[1]->VisibilityOff();
    labels_[1]->VisibilityOff();
    labels_[2]->VisibilityOff();
    lineActor_->VisibilityOff();
    lineOutline_->VisibilityOff();
    result_.success = false;
    if (!selecting_) emit resultChanged(QString());
    render();
}

void ViewportPointPicker::updateAnnotations()
{
    const bool visible = points_[0]->GetVisibility() || points_[1]->GetVisibility();
    overlayRenderer_->SetDraw(visible);
    if (!visible) return;
    const int* size = window_->GetSize();
    if (size[1] <= 0) return;
    auto camera = renderer_->GetActiveCamera();
    overlayRenderer_->SetActiveCamera(camera);
    overlayRenderer_->SetViewport(renderer_->GetViewport());
    const double scale = view_ && view_->height() > 0 ? double(size[1]) / view_->height() : 1.0;
    auto pixelSize = [&](const double* point) {
        double worldHeight = 2.0 * camera->GetParallelScale();
        if (!camera->GetParallelProjection()) {
            double direction[3];
            camera->GetDirectionOfProjection(direction);
            const double* position = camera->GetPosition();
            double depth = 0.0;
            for (int i = 0; i < 3; ++i) depth += (point[i] - position[i]) * direction[i];
            worldHeight = 2.0 * std::max(depth, 1e-9) * std::tan(camera->GetViewAngle() * 0.008726646259971648);
        }
        return std::max(worldHeight / size[1], 1e-12);
    };
    for (int i = 0; i < 2; ++i) {
        if (points_[i]->GetVisibility()) spheres_[i]->SetRadius(pixelSize(spheres_[i]->GetCenter()) * 5.0 * scale);
    }
    for (int i = 0; i < 3; ++i) {
        if (!labels_[i]->GetVisibility()) continue;
        const int pixels = int(std::clamp(textHeight_ / pixelSize(labels_[i]->GetPosition()), 16.0 * scale, 28.0 * scale));
        const double pointsPerPixel = 72.0 / std::max(window_->GetDPI(), 1);
        labels_[i]->GetTextProperty()->SetFontSize(std::max(1, qRound((i == 2 ? pixels : 16 * scale) * pointsPerPixel)));
        labels_[i]->SetDisplayOffset(i == 2 ? 0 : int(10 * scale), int((i == 2 ? 30 : 10) * scale));
    }
    double direction[2] = {1.0, 0.0};
    if (points_[0]->GetVisibility() && points_[1]->GetVisibility()) {
        double projected[2][3];
        for (int i = 0; i < 2; ++i) {
            auto p = spheres_[i]->GetCenter();
            renderer_->SetWorldPoint(p[0], p[1], p[2], 1.0);
            renderer_->WorldToDisplay();
            renderer_->GetDisplayPoint(projected[i]);
        }
        const double dx = projected[1][0] - projected[0][0];
        const double dy = projected[1][1] - projected[0][1];
        const double length = std::hypot(dx, dy);
        if (length > 1.0) { direction[0] = dx / length; direction[1] = dy / length; }
    }
    for (int i = 0; i < 2; ++i) {
        const double sign = i == 0 ? -1.0 : 1.0;
        const int x = qRound(sign * (direction[0] * 16 + direction[1] * 12) * scale);
        const int y = qRound(sign * (direction[1] * 16 - direction[0] * 12) * scale);
        labels_[i]->SetDisplayOffset(x, y);
        labels_[i]->GetTextProperty()->SetJustification(x < 0 ? VTK_TEXT_RIGHT : VTK_TEXT_LEFT);
        labels_[i]->GetTextProperty()->SetVerticalJustificationToCentered();
    }
}

QString ViewportPointPicker::resultText() const
{
    return QString("空间距离: %1 mm\nA: (%2, %3, %4) mm\nB: (%5, %6, %7) mm")
        .arg(result_.distance, 0, 'f', 3)
        .arg(result_.start[0], 0, 'f', 3).arg(result_.start[1], 0, 'f', 3).arg(result_.start[2], 0, 'f', 3)
        .arg(result_.end[0], 0, 'f', 3).arg(result_.end[1], 0, 'f', 3).arg(result_.end[2], 0, 'f', 3);
}

void ViewportPointPicker::setHint(const QString& text)
{
    hint_->setText(text);
    hint_->ensurePolished();
    const int available = view_ ? std::max(1, view_->width() - 104) : 600;
    const int width = std::min(available, QFontMetrics(hint_->font()).horizontalAdvance(text) + 28);
    hint_->setWordWrap(true);
    hint_->setFixedSize(width, hint_->heightForWidth(width));
    hint_->show();
    hint_->raise();
    cancel_->adjustSize();
    cancel_->move(hint_->geometry().right() + 8, hint_->y());
    cancel_->show();
    cancel_->raise();
}

void ViewportPointPicker::render()
{
    if (view_ && view_->isVisible()) {
        window_->Render();
        view_->update();
    }
}
