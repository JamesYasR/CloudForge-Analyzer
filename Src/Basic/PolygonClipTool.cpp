#include "Basic/PolygonClipTool.h"
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkIdTypeArray.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <algorithm>
#include <cmath>
#include <functional>

namespace {
class ContourCanvas : public QWidget
{
public:
    ContourCanvas(QWidget* parent, std::function<void(QPainter&)> paint) : QWidget(parent), paint_(std::move(paint))
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
        setObjectName("polygonClipCanvas");
    }
protected:
    void paintEvent(QPaintEvent*) override { QPainter painter(this); paint_(painter); }
private:
    std::function<void(QPainter&)> paint_;
};
}

PolygonClipTool::PolygonClipTool(QWidget* view, vtkRenderer* renderer, vtkRenderWindow* window)
    : view_(view), renderer_(renderer), window_(window)
{
    canvas_ = new ContourCanvas(view, [this](QPainter& painter) { paintContour(painter); });
    canvas_->hide();
    buildPanel();
    renderTimer_.setSingleShot(true);
    connect(&renderTimer_, &QTimer::timeout, this, [this]() {
        if (view_ && view_->isVisible()) { window_->Render(); view_->update(); }
    });
    viewTimer_.setInterval(100);
    connect(&viewTimer_, &QTimer::timeout, this, [this]() {
        if (result_ && !(projection() == pendingProjection_)) {
            invalidatePreview();
            showMessage("视角已变化，请重新预览");
        }
    });
    qApp->installEventFilter(this);
}

PolygonClipTool::~PolygonClipTool()
{
    stop();
    qApp->removeEventFilter(this);
    delete panel_;
    delete canvas_;
}

void PolygonClipTool::buildPanel()
{
    panel_ = new QFrame(view_);
    panel_->setObjectName("polygonClipPanel");
    panel_->setStyleSheet(
        "QFrame#polygonClipPanel { background: #102e39; border: 1px solid #41606a; border-radius: 10px; }"
        "QLabel { color: #e3f0f2; border: none; background: transparent; }"
        "QPushButton { color: #e3f0f2; background: #244651; border: none; border-radius: 5px; padding: 6px 9px; }"
        "QPushButton:hover { background: #345e69; } QPushButton:checked { background: #18786e; }"
        "QPushButton:disabled { color: #69858c; background: #1c3943; }"
        "QPushButton#polygonApply { color: #062c29; background: #48d7b2; font-weight: bold; }"
        "QPushButton#polygonApply:disabled { color: #69858c; background: #1c3943; }"
        "QComboBox { color: #ecf5f6; background: #244651; border: 1px solid #41606a; border-radius: 4px; padding: 4px 7px; }"
        "QComboBox QAbstractItemView { color: #ecf5f6; background: #244651; selection-background-color: #18786e; }"
        "QCheckBox { color: #adc4c9; background: transparent; }"
    );
    auto layout = new QVBoxLayout(panel_);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(8);
    auto header = new QHBoxLayout;
    auto title = new QLabel("多边形裁切", panel_);
    title->setStyleSheet("font-size: 15px; font-weight: bold; color: #74e2c5;");
    header->addWidget(title, 1);
    auto collapse = new QPushButton("−", panel_);
    collapse->setObjectName("polygonCollapse");
    collapse->setFixedWidth(30);
    collapse->setToolTip("收起 / 展开操作面板");
    header->addWidget(collapse);
    auto cancel = new QPushButton("×", panel_);
    cancel->setObjectName("polygonCancel");
    cancel->setFixedWidth(30);
    cancel->setToolTip("取消裁切（Esc / Q）");
    header->addWidget(cancel);
    layout->addLayout(header);
    body_ = new QWidget(panel_);
    auto body = new QVBoxLayout(body_);
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(8);
    source_ = new QComboBox(body_);
    source_->setObjectName("polygonSource");
    source_->setToolTip("选择要裁切的点云");
    body->addWidget(source_);
    auto modes = new QHBoxLayout;
    draw_ = new QPushButton("绘制轮廓", body_);
    navigate_ = new QPushButton("调整视角", body_);
    draw_->setObjectName("polygonDraw");
    navigate_->setObjectName("polygonNavigate");
    auto group = new QButtonGroup(this);
    for (auto button : {draw_, navigate_}) { button->setCheckable(true); group->addButton(button); modes->addWidget(button); }
    draw_->setToolTip("单击描边，拖动顶点修改；X 切换模式");
    navigate_->setToolTip("使用原有旋转、平移和缩放；X 切换模式");
    body->addLayout(modes);
    hint_ = new QLabel(body_);
    hint_->setObjectName("polygonHint");
    hint_->setWordWrap(true);
    hint_->setMinimumHeight(34);
    hint_->setStyleSheet("color: #adc4c9; font-size: 12px;");
    body->addWidget(hint_);
    auto edit = new QHBoxLayout;
    close_ = new QPushButton("闭合轮廓", body_);
    undo_ = new QPushButton("撤回", body_);
    reset_ = new QPushButton("重画", body_);
    close_->setObjectName("polygonClose");
    undo_->setObjectName("polygonUndo");
    reset_->setObjectName("polygonReset");
    close_->setToolTip("闭合轮廓（Enter / 右键）");
    undo_->setToolTip("撤回上一步轮廓修改（Backspace / Ctrl+Z）");
    reset_->setToolTip("重新绘制（R）");
    for (auto button : {close_, undo_, reset_}) edit->addWidget(button);
    body->addLayout(edit);
    mode_ = new QComboBox(body_);
    mode_->setObjectName("polygonMode");
    mode_->addItems({"保留框内", "保留框外", "分成两份"});
    body->addWidget(mode_);
    count_ = new QLabel("描边后预览裁切结果", body_);
    count_->setObjectName("polygonCount");
    count_->setWordWrap(true);
    count_->setStyleSheet("color: #74e2c5; font-size: 12px;");
    body->addWidget(count_);
    auto actions = new QHBoxLayout;
    preview_ = new QPushButton("预览裁切", body_);
    apply_ = new QPushButton("应用裁切", body_);
    preview_->setObjectName("polygonPreview");
    apply_->setObjectName("polygonApply");
    preview_->setToolTip("预览裁切（C）");
    apply_->setToolTip("应用当前预览；可通过主界面撤销还原");
    actions->addWidget(preview_);
    actions->addWidget(apply_);
    body->addLayout(actions);
    auto footer = new QHBoxLayout;
    contourVisible_ = new QCheckBox("显示轮廓", body_);
    contourVisible_->setObjectName("polygonContourVisible");
    contourVisible_->setChecked(true);
    contourVisible_->setToolTip("显示 / 隐藏轮廓（S）");
    footer->addWidget(contourVisible_, 1);
    auto exit = new QPushButton("取消", body_);
    exit->setObjectName("polygonExit");
    footer->addWidget(exit);
    body->addLayout(footer);
    layout->addWidget(body_);
    panel_->setFixedWidth(306);
    panel_->move(12, 12);
    panel_->hide();
    connect(cancel, &QPushButton::clicked, this, &PolygonClipTool::stop);
    connect(exit, &QPushButton::clicked, this, &PolygonClipTool::stop);
    connect(collapse, &QPushButton::clicked, this, [this]() {
        body_->setVisible(!body_->isVisible());
        panel_->adjustSize();
    });
    connect(draw_, &QPushButton::clicked, this, [this]() { setDrawing(true); });
    connect(navigate_, &QPushButton::clicked, this, [this]() { setDrawing(false); });
    connect(close_, &QPushButton::clicked, this, &PolygonClipTool::closePolygon);
    connect(undo_, &QPushButton::clicked, this, &PolygonClipTool::undoVertex);
    connect(reset_, &QPushButton::clicked, this, &PolygonClipTool::resetPolygon);
    connect(preview_, &QPushButton::clicked, this, &PolygonClipTool::preview);
    connect(apply_, &QPushButton::clicked, this, [this]() { if (canApply()) emit applyRequested(); });
    connect(contourVisible_, &QCheckBox::toggled, canvas_, qOverload<>(&QWidget::update));
    connect(mode_, &QComboBox::currentIndexChanged, this, [this]() { updatePreviewColors(); updateUi(); });
    connect(source_, &QComboBox::currentIndexChanged, this, [this]() {
        clearPreview();
        if (active_) originalVisibility_ = target().actor->GetVisibility();
        resetPolygon();
        history_.clear();
        updateUi();
    });
}

bool PolygonClipTool::start(const std::vector<Target>& targets)
{
    stop();
    targets_.clear();
    for (const auto& target : targets) {
        if (target.actor && target.cloud && !target.cloud->empty() && target.actor->GetVisibility()
            && target.actor->GetProperty()->GetOpacity() > 0) targets_.push_back(target);
    }
    if (targets_.empty() || !view_) return false;
    { QSignalBlocker blocker(source_); source_->clear(); for (const auto& target : targets_) source_->addItem(QString::fromStdString(target.name)); }
    active_ = true;
    busy_ = false;
    drawing_ = true;
    pressed_ = false;
    closed_ = false;
    polygon_.clear();
    history_.clear();
    selected_ = drag_ = -1;
    originalVisibility_ = target().actor->GetVisibility();
    originalCursor_ = view_->cursor();
    mouseTracking_ = view_->hasMouseTracking();
    view_->setMouseTracking(true);
    view_->setCursor(Qt::CrossCursor);
    view_->setFocus(Qt::OtherFocusReason);
    canvas_->setGeometry(view_->rect());
    canvas_->show();
    canvas_->raise();
    body_->show();
    panel_->setFixedWidth(std::min(306, view_->width() - 24));
    panel_->adjustSize();
    panel_->show();
    panel_->raise();
    contourVisible_->setChecked(true);
    viewTimer_.start();
    updateUi();
    showMessage("单击描边，点击起点或闭合按钮完成");
    emit activeChanged(true);
    return true;
}

void PolygonClipTool::stop()
{
    if (!active_) return;
    const bool cancel = busy_;
    active_ = busy_ = pressed_ = false;
    ++ticket_;
    viewTimer_.stop();
    clearPreview();
    panel_->hide();
    canvas_->hide();
    if (view_) { view_->setCursor(originalCursor_); view_->setMouseTracking(mouseTracking_); }
    emit activeChanged(false);
    if (cancel) emit cancelPreviewRequested();
}

const PolygonClipTool::Target& PolygonClipTool::target() const { return targets_[std::max(source_->currentIndex(), 0)]; }
PolygonClipTool::KeepMode PolygonClipTool::keepMode() const { return static_cast<KeepMode>(mode_->currentIndex()); }

CloudClipper::Projection PolygonClipTool::projection() const
{
    CloudClipper::Projection output;
    auto camera = renderer_->GetActiveCamera();
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkMatrix4x4::Multiply4x4(camera->GetCompositeProjectionTransformMatrix(renderer_->GetTiledAspectRatio(), 0, 1),
                            target().actor->GetMatrix(), matrix);
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) output.matrix[row * 4 + col] = matrix->GetElement(row, col);
    std::copy_n(renderer_->GetViewport(), 4, output.viewport.begin());
    return output;
}

CloudClipper::Request PolygonClipTool::request() const { return {target().cloud, polygon_, pendingProjection_}; }

void PolygonClipTool::preview()
{
    if (!active_ || busy_ || !closed_) return;
    auto error = CloudClipper::validatePolygon(polygon_);
    if (!error.empty()) { showMessage(QString::fromStdString(error)); return; }
    clearPreview();
    pendingProjection_ = projection();
    busy_ = true;
    ++ticket_;
    updateUi();
    showMessage("正在预览裁切…");
    emit previewRequested();
}

void PolygonClipTool::finishPreview(const std::shared_ptr<CloudClipper::Result>& result, unsigned long ticket)
{
    if (!active_ || ticket != ticket_) return;
    busy_ = false;
    if (!(projection() == pendingProjection_)) { updateUi(); showMessage("视角已变化，请重新预览"); return; }
    if (!result->success || result->cancelled) {
        updateUi();
        showMessage(result->cancelled ? "预览已取消" : QString::fromStdString(result->error));
        return;
    }
    result_ = result;
    insideActor_ = previewActor(*result->inside);
    outsideActor_ = previewActor(*result->outside);
    previewSource_ = target().actor;
    previewSource_->VisibilityOff();
    updatePreviewColors();
    updateUi();
    if (canApply()) showMessage("拖动顶点可修改轮廓，满意后应用裁切");
    else showMessage(keepMode() == KeepMode::Both ? "轮廓未分开点云，请调整轮廓" : "要保留的区域没有点，请调整轮廓");
    render();
}

bool PolygonClipTool::canApply()
{
    if (!active_ || busy_ || !result_) return false;
    if (!(projection() == pendingProjection_)) { invalidatePreview(); showMessage("视角已变化，请重新预览"); return false; }
    const bool valid = keepMode() == KeepMode::Inside ? !result_->inside->empty()
        : keepMode() == KeepMode::Outside ? !result_->outside->empty()
        : !result_->inside->empty() && !result_->outside->empty();
    return valid;
}

vtkSmartPointer<vtkActor> PolygonClipTool::previewActor(const CloudClipper::Cloud& cloud)
{
    auto coordinates = vtkSmartPointer<vtkFloatArray>::New();
    coordinates->SetNumberOfComponents(3);
    coordinates->SetNumberOfTuples(cloud.size());
    auto ids = vtkSmartPointer<vtkIdTypeArray>::New();
    ids->SetNumberOfValues(cloud.size());
    float* values = coordinates->GetPointer(0);
    vtkIdType* indices = ids->GetPointer(0);
    for (size_t i = 0; i < cloud.size(); ++i) {
        values[i * 3] = cloud[i].x; values[i * 3 + 1] = cloud[i].y; values[i * 3 + 2] = cloud[i].z;
        indices[i] = static_cast<vtkIdType>(i);
    }
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetData(coordinates);
    auto vertices = vtkSmartPointer<vtkCellArray>::New();
    vertices->SetData(1, ids);
    auto data = vtkSmartPointer<vtkPolyData>::New();
    data->SetPoints(points);
    data->SetVerts(vertices);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(data);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    auto transform = vtkSmartPointer<vtkMatrix4x4>::New();
    transform->DeepCopy(target().actor->GetMatrix());
    actor->SetUserMatrix(transform);
    actor->PickableOff();
    actor->UseBoundsOff();
    actor->GetProperty()->SetPointSize(std::max<double>(target().actor->GetProperty()->GetPointSize(), 2.0));
    actor->GetProperty()->LightingOff();
    renderer_->AddActor(actor);
    return actor;
}

void PolygonClipTool::clearPreview()
{
    const bool changed = insideActor_ || outsideActor_;
    if (insideActor_ || outsideActor_) {
        if (previewSource_) previewSource_->SetVisibility(originalVisibility_);
        if (insideActor_) renderer_->RemoveActor(insideActor_);
        if (outsideActor_) renderer_->RemoveActor(outsideActor_);
        insideActor_ = nullptr;
        outsideActor_ = nullptr;
        previewSource_ = nullptr;
    }
    result_.reset();
    if (changed) render();
}

void PolygonClipTool::invalidatePreview()
{
    if (busy_) return;
    clearPreview();
    updateUi();
    canvas_->update();
}

void PolygonClipTool::updatePreviewColors()
{
    if (!result_) return;
    const bool both = keepMode() == KeepMode::Both;
    insideActor_->GetProperty()->SetColor(0.28, 0.93, 0.74);
    outsideActor_->GetProperty()->SetColor(both ? 1.0 : 0.95, both ? 0.65 : 0.35, both ? 0.27 : 0.35);
    insideActor_->GetProperty()->SetOpacity(keepMode() == KeepMode::Outside ? 0.2 : 1.0);
    outsideActor_->GetProperty()->SetOpacity(keepMode() == KeepMode::Inside ? 0.2 : 1.0);
    render();
}

void PolygonClipTool::setDrawing(bool drawing)
{
    if (busy_ || QApplication::mouseButtons() != Qt::NoButton) return;
    drawing_ = drawing;
    pressed_ = false;
    drag_ = -1;
    view_->setCursor(drawing ? Qt::CrossCursor : originalCursor_);
    updateUi();
    showMessage(drawing ? "单击描边，拖动顶点或点击边线插入顶点" : "拖动调整视角，完成后切回绘制轮廓");
}

void PolygonClipTool::closePolygon()
{
    if (busy_ || closed_ || polygon_.size() < 3) return;
    rememberContour();
    closed_ = true;
    updateUi();
    auto error = CloudClipper::validatePolygon(polygon_);
    showMessage(error.empty() ? "轮廓已闭合，拖动顶点调整或预览裁切" : QString::fromStdString(error));
    canvas_->update();
}

void PolygonClipTool::undoVertex()
{
    if (busy_ || history_.empty()) return;
    invalidatePreview();
    polygon_ = std::move(history_.back().polygon);
    closed_ = history_.back().closed;
    history_.pop_back();
    selected_ = drag_ = -1;
    updateUi();
    showMessage(closed_ ? "轮廓已还原，请预览裁切" : "继续单击描边，或点击起点闭合");
    canvas_->update();
}

void PolygonClipTool::resetPolygon()
{
    if (busy_) return;
    if (!polygon_.empty()) rememberContour();
    invalidatePreview();
    polygon_.clear();
    closed_ = pressed_ = false;
    selected_ = drag_ = -1;
    updateUi();
    showMessage("单击描边，点击起点或闭合按钮完成");
    canvas_->update();
}

void PolygonClipTool::rememberContour()
{
    history_.push_back({polygon_, closed_});
    if (history_.size() > 64) history_.erase(history_.begin());
}

void PolygonClipTool::updateUi()
{
    const bool valid = closed_ && CloudClipper::validatePolygon(polygon_).empty();
    source_->setEnabled(!busy_);
    draw_->setEnabled(!busy_);
    navigate_->setEnabled(!busy_);
    draw_->setChecked(drawing_);
    navigate_->setChecked(!drawing_);
    close_->setEnabled(!busy_ && !closed_ && polygon_.size() >= 3);
    undo_->setEnabled(!busy_ && !history_.empty());
    reset_->setEnabled(!busy_ && !polygon_.empty());
    mode_->setEnabled(!busy_);
    preview_->setEnabled(!busy_ && valid);
    apply_->setEnabled(!busy_ && result_ && (keepMode() == KeepMode::Inside ? !result_->inside->empty()
        : keepMode() == KeepMode::Outside ? !result_->outside->empty() : !result_->inside->empty() && !result_->outside->empty()));
    count_->setText(result_ ? QString("<span style='color:#74e2c5'>框内 %1 点</span>　·　<span style='color:#ffb18e'>框外 %2 点</span>").arg(result_->inside->size()).arg(result_->outside->size())
                           : "描边后预览裁切结果");
}

void PolygonClipTool::showMessage(const QString& message) { hint_->setText(message); }
QPointF PolygonClipTool::normalized(const QPointF& point) const
{
    return {std::clamp(point.x() / view_->width(), 0.0, 1.0), std::clamp(point.y() / view_->height(), 0.0, 1.0)};
}
QPointF PolygonClipTool::displayed(const CloudClipper::Point2D& point) const { return {point[0] * view_->width(), point[1] * view_->height()}; }

int PolygonClipTool::hitVertex(const QPointF& point) const
{
    for (size_t i = 0; i < polygon_.size(); ++i) if (QLineF(displayed(polygon_[i]), point).length() <= 11) return int(i);
    return -1;
}

int PolygonClipTool::hitEdge(const QPointF& point) const
{
    for (size_t i = 0; i < polygon_.size(); ++i) {
        const QPointF a = displayed(polygon_[i]), b = displayed(polygon_[(i + 1) % polygon_.size()]);
        const QPointF delta = b - a;
        const double length = QPointF::dotProduct(delta, delta);
        if (length <= 0) continue;
        const double t = std::clamp(QPointF::dotProduct(point - a, delta) / length, 0.0, 1.0);
        if (QLineF(a + delta * t, point).length() <= 7) return int(i);
    }
    return -1;
}

bool PolygonClipTool::eventFilter(QObject* watched, QEvent* event)
{
    if (!active_ || !view_) return false;
    if (event->type() == QEvent::ShortcutOverride) {
        auto widget = qobject_cast<QWidget*>(watched);
        auto key = static_cast<QKeyEvent*>(event);
        if (widget && widget->window() == view_->window() && key->key() == Qt::Key_Z
            && (key->modifiers() & Qt::ControlModifier)) {
            event->accept();
            return true;
        }
    }
    if (event->type() == QEvent::KeyPress) {
        auto widget = qobject_cast<QWidget*>(watched);
        auto key = static_cast<QKeyEvent*>(event);
        if (!widget || widget->window() != view_->window()) return false;
        if (key->key() == Qt::Key_Escape) { stop(); return true; }
        if (widget == source_ || widget == mode_) return false;
        switch (key->key()) {
        case Qt::Key_Q: stop(); return true;
        case Qt::Key_X: setDrawing(!drawing_); return true;
        case Qt::Key_R: resetPolygon(); return true;
        case Qt::Key_C: preview(); return true;
        case Qt::Key_S: contourVisible_->toggle(); return true;
        case Qt::Key_Return: case Qt::Key_Enter: closePolygon(); return true;
        case Qt::Key_Backspace: undoVertex(); return true;
        case Qt::Key_Z: if (key->modifiers() & Qt::ControlModifier) { undoVertex(); return true; } break;
        default: break;
        }
    }
    if (watched != view_) return false;
    if (event->type() == QEvent::Resize) {
        canvas_->setGeometry(view_->rect());
        panel_->setFixedWidth(std::min(306, view_->width() - 24));
        panel_->adjustSize();
        invalidatePreview();
        return false;
    }
    if (!drawing_ && !busy_) {
        if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::Wheel) invalidatePreview();
        return false;
    }
    if (event->type() == QEvent::Wheel) return true;
    if (event->type() == QEvent::MouseButtonDblClick) { closePolygon(); pressed_ = false; return true; }
    if (event->type() == QEvent::MouseButtonPress) {
        auto mouse = static_cast<QMouseEvent*>(event);
        if (busy_) return true;
        if (mouse->button() == Qt::RightButton) {
            if (!closed_) closePolygon();
            else {
                const int index = hitVertex(mouse->position());
                auto menu = new QMenu(view_);
                menu->setAttribute(Qt::WA_DeleteOnClose);
                if (index >= 0) menu->addAction("删除这个顶点", this, [this, index]() {
                    if (!active_ || busy_ || index >= int(polygon_.size())) return;
                    rememberContour();
                    invalidatePreview(); polygon_.erase(polygon_.begin() + index); closed_ = polygon_.size() >= 3;
                    selected_ = -1; updateUi(); canvas_->update();
                });
                menu->addAction("重新绘制", this, &PolygonClipTool::resetPolygon);
                menu->addAction("预览裁切", this, &PolygonClipTool::preview);
                menu->popup(mouse->globalPosition().toPoint());
            }
        }
        else if (mouse->button() == Qt::LeftButton) {
            pressed_ = true;
            moved_ = false;
            gestureSaved_ = false;
            press_ = cursor_ = mouse->position();
            drag_ = selected_ = hitVertex(cursor_);
            closeOnRelease_ = drag_ == 0 && !closed_ && polygon_.size() >= 3;
            if (drag_ < 0 && closed_) {
                const int edge = hitEdge(cursor_);
                if (edge >= 0) {
                    rememberContour();
                    gestureSaved_ = true;
                    invalidatePreview();
                    const auto point = normalized(cursor_);
                    polygon_.insert(polygon_.begin() + edge + 1, {point.x(), point.y()});
                    drag_ = selected_ = edge + 1;
                }
            }
        }
        return true;
    }
    if (event->type() == QEvent::MouseMove) {
        auto mouse = static_cast<QMouseEvent*>(event);
        cursor_ = mouse->position();
        if (pressed_ && QLineF(cursor_, press_).length() >= 3) moved_ = true;
        if (!busy_ && pressed_ && drag_ >= 0 && moved_) {
            if (!gestureSaved_) { rememberContour(); gestureSaved_ = true; }
            invalidatePreview();
            const auto point = normalized(cursor_);
            polygon_[drag_] = {point.x(), point.y()};
            updateUi();
            auto error = closed_ ? CloudClipper::validatePolygon(polygon_) : std::string();
            showMessage(error.empty() ? "拖动顶点调整，完成后预览裁切" : QString::fromStdString(error));
        }
        canvas_->update();
        return true;
    }
    if (event->type() == QEvent::MouseButtonRelease) {
        auto mouse = static_cast<QMouseEvent*>(event);
        if (!busy_ && mouse->button() == Qt::LeftButton && pressed_) {
            if (closeOnRelease_ && !moved_) closePolygon();
            else if (!closed_ && drag_ < 0 && !moved_) {
                rememberContour();
                const auto point = normalized(mouse->position());
                polygon_.push_back({point.x(), point.y()});
            }
            pressed_ = false;
            drag_ = -1;
            updateUi();
            canvas_->update();
        }
        return true;
    }
    if (event->type() == QEvent::Leave) { cursor_ = {-100, -100}; canvas_->update(); }
    return false;
}

void PolygonClipTool::paintContour(QPainter& painter)
{
    if (!active_ || polygon_.empty() || !contourVisible_->isChecked()) return;
    painter.setRenderHint(QPainter::Antialiasing);
    QPolygonF points;
    for (const auto& point : polygon_) points << displayed(point);
    const bool invalid = closed_ && !CloudClipper::validatePolygon(polygon_).empty();
    const QColor color = invalid ? QColor("#ff9292") : QColor("#6cf2cb");
    if (closed_) {
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 28));
        painter.setPen(QPen(QColor(5, 29, 36, 220), 5));
        painter.drawPolygon(points);
    }
    painter.setPen(QPen(color, 2));
    painter.setBrush(Qt::NoBrush);
    if (closed_) painter.drawPolygon(points); else painter.drawPolyline(points);
    if (!closed_ && drawing_ && cursor_.x() >= 0) {
        painter.setPen(QPen(color, 1.5, Qt::DashLine));
        painter.drawLine(points.back(), cursor_);
        if (points.size() >= 2) painter.drawLine(cursor_, points.front());
    }
    for (int i = 0; i < points.size(); ++i) {
        const bool close = i == 0 && !closed_ && points.size() >= 3 && QLineF(cursor_, points[0]).length() <= 11;
        painter.setPen(QPen(QColor("#0d3038"), 2));
        painter.setBrush(i == selected_ || close ? QColor("#ffffff") : color);
        painter.drawEllipse(points[i], close ? 8 : 5, close ? 8 : 5);
        painter.setPen(QColor("#ffffff"));
        painter.drawText(points[i] + QPointF(9, -9), close ? "点击闭合" : QString::number(i + 1));
    }
}

void PolygonClipTool::render()
{
    if (view_ && view_->isVisible() && !renderTimer_.isActive()) renderTimer_.start(0);
}
