#include "Basic/ViewportCloudSelector.h"
#include <QAction>
#include <QApplication>
#include <QDialog>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWidget>
#include <vtkActor.h>
#include <vtkOutlineSource.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <cmath>

ViewportCloudSelector::ViewportCloudSelector(QWidget* view, vtkRenderer* renderer,
        vtkRenderWindow* window, ReadActors read, QObject* parent)
    : QObject(parent), view_(view), renderer_(renderer), window_(window), read_(std::move(read)),
      picker_(vtkSmartPointer<vtkPointPicker>::New())
{
    picker_->PickFromListOn();
    picker_->UseCellsOff();
}

ViewportCloudSelector::~ViewportCloudSelector() { end(dialog_); }

bool ViewportCloudSelector::begin(QDialog* dialog, const std::set<std::string>& eligible)
{
    if (dialog_ || !view_ || !dialog) return false;
    dialog_ = dialog;
    actors_.clear();
    for (const auto& entry : read_()) {
        if (eligible.count(entry.first)) actors_.emplace(entry);
    }
    emit selectionStarted();
    // 保留原有同步对话框流程，但只放行视图中的点选/相机操作，禁止后台改动场景。
    for (auto* action : view_->window()->findChildren<QAction*>()) {
        actions_.push_back({action, action->isEnabled()});
        action->setEnabled(false);
    }
    qApp->installEventFilter(this);
    return true;
}

void ViewportCloudSelector::end(QDialog* dialog)
{
    if (!dialog_ || dialog_ != dialog) return;
    setPicking(false);
    qApp->removeEventFilter(this);
    for (auto& action : actions_) if (action.first) action.first->setEnabled(action.second);
    actions_.clear();
    picker_->InitializePickList();
    actors_.clear();
    selected_.clear();
    dialog_.clear();
}

void ViewportCloudSelector::setPicking(bool enabled)
{
    if (!dialog_ || !view_ || picking_ == enabled) return;
    picking_ = enabled;
    pressed_ = dragged_ = false;
    if (enabled) {
        cursor_ = view_->cursor();
        view_->setCursor(Qt::CrossCursor);
    } else view_->setCursor(cursor_);
    updateHighlights();
    emit pickingChanged(enabled);
}

void ViewportCloudSelector::setSelected(const std::set<std::string>& selected)
{
    selected_ = selected;
    updateHighlights();
}

void ViewportCloudSelector::updateHighlights()
{
    for (const auto& actor : highlights_) renderer_->RemoveActor(actor);
    highlights_.clear();
    if (picking_) {
        for (const auto& id : selected_) {
            auto found = actors_.find(id);
            if (found == actors_.end() || !found->second || !found->second->GetVisibility()) continue;
            auto* actor = found->second.GetPointer();
            const auto* bounds = actor->GetBounds();
            if (actor->GetProperty()->GetOpacity() <= 0 || !bounds || bounds[0] > bounds[1]) continue;
            auto outline = vtkSmartPointer<vtkOutlineSource>::New();
            outline->SetBounds(actor->GetBounds());
            auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(outline->GetOutputPort());
            auto highlight = vtkSmartPointer<vtkActor>::New();
            highlight->SetMapper(mapper);
            highlight->GetProperty()->SetColor(1.0, 0.85, 0.15);
            highlight->GetProperty()->SetLineWidth(2.0);
            highlight->GetProperty()->LightingOff();
            highlight->PickableOff();
            highlight->UseBoundsOff();
            renderer_->AddActor(highlight);
            highlights_.push_back(highlight);
        }
    }
    if (view_ && view_->isVisible()) { window_->Render(); view_->update(); }
}

void ViewportCloudSelector::pick(const QPoint& position)
{
    if (!view_ || !view_->rect().contains(position)) return;
    const auto* size = window_->GetSize();
    if (size[0] <= 0 || size[1] <= 0 || view_->width() <= 0 || view_->height() <= 0) return;
    picker_->InitializePickList();
    for (const auto& entry : actors_) {
        auto* actor = entry.second.GetPointer();
        if (actor && actor->GetVisibility() && actor->GetPickable() && actor->GetProperty()->GetOpacity() > 0)
            picker_->AddPickList(actor);
    }
    const double scale = double(size[0]) / view_->width();
    picker_->SetTolerance(6.0 * scale / std::hypot(size[0], size[1]));
    if (!picker_->Pick(position.x() * scale, size[1] - 1 - position.y() * double(size[1]) / view_->height(),
                       0.0, renderer_) || picker_->GetPointId() < 0) return;
    for (const auto& entry : actors_) {
        if (entry.second.GetPointer() == picker_->GetActor()) {
            emit cloudClicked(QString::fromStdString(entry.first));
            break;
        }
    }
}

bool ViewportCloudSelector::eventFilter(QObject* watched, QEvent* event)
{
    if (!dialog_ || !view_) return false;
    if (watched == view_->window() && event->type() == QEvent::Close) {
        dialog_->reject();
        return false;
    }
    auto* widget = qobject_cast<QWidget*>(watched);
    if (!widget || widget == dialog_ || dialog_->isAncestorOf(widget)) return false;
    auto* host = view_->window();
    if (widget != host && !host->isAncestorOf(widget)) return false;
    if (event->type() == QEvent::KeyPress && picking_ &&
        static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
        setPicking(false);
        dialog_->raise();
        dialog_->activateWindow();
        return true;
    }
    const bool mouseEvent = event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonRelease ||
        event->type() == QEvent::MouseButtonDblClick || event->type() == QEvent::MouseMove || event->type() == QEvent::Wheel;
    if (!mouseEvent) {
        return event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease ||
               event->type() == QEvent::ShortcutOverride || event->type() == QEvent::ContextMenu;
    }
    if (!picking_ || watched != view_) return true;
    if (event->type() == QEvent::MouseButtonPress) {
        auto* mouse = static_cast<QMouseEvent*>(event);
        pressed_ = mouse->button() == Qt::LeftButton && mouse->modifiers() == Qt::NoModifier;
        dragged_ = false;
        press_ = mouse->position().toPoint();
    } else if (event->type() == QEvent::MouseMove && pressed_) {
        if ((static_cast<QMouseEvent*>(event)->position().toPoint() - press_).manhattanLength() >= QApplication::startDragDistance())
            dragged_ = true;
    } else if (event->type() == QEvent::MouseButtonRelease) {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
            const bool click = pressed_ && !dragged_ && mouse->modifiers() == Qt::NoModifier &&
                (mouse->position().toPoint() - press_).manhattanLength() < QApplication::startDragDistance();
            pressed_ = false;
            if (click) pick(mouse->position().toPoint());
        }
    } else if (event->type() == QEvent::Wheel) pressed_ = false;
    return false; // 保留拖动旋转和滚轮缩放。
}
