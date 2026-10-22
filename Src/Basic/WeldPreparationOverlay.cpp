#include "Basic/WeldPreparationOverlay.h"
#include "Basic/ViewportResultOverlay.h"

WeldPreparationOverlay::WeldPreparationOverlay(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results)
    : ViewportAnnotationOverlay(view,renderer,results,true), results_(results)
{
    setObjectName("weldPreparationOverlay");
}

void WeldPreparationOverlay::setAnnotations(std::vector<Annotation> annotations)
{
    if (results_) {
        if (annotations.empty()) results_->removeSection("weld_preparation");
        else results_->setSection("weld_preparation", {{}, {
            {"近件缝边", {}, QColor("#42f294")}, {"远件缝边", {}, QColor("#ffda55")},
            {"径向阶差", {}, QColor("#68b7ff")}, {"轴向间隙", {}, QColor("#ff6b65")}
        }, 2});
    }
    ViewportAnnotationOverlay::setAnnotations(std::move(annotations));
}

void WeldPreparationOverlay::clear()
{
    if (results_) results_->removeSection("weld_preparation");
    ViewportAnnotationOverlay::clear();
}
