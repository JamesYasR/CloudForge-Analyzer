#pragma once
#include "Basic/ViewportAnnotationOverlay.h"

class WeldPreparationOverlay : public ViewportAnnotationOverlay
{
public:
    WeldPreparationOverlay(QWidget* view, vtkRenderer* renderer, ViewportResultOverlay* results = nullptr);
    void setAnnotations(std::vector<Annotation> annotations);
    void clear();
private:
    ViewportResultOverlay* results_ = nullptr;
};
