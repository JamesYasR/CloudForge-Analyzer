#include "Basic/MeasurementPresentation.h"
#include "Basic/ViewportResultOverlay.h"
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkUnsignedCharArray.h>
#include <algorithm>

MeasurementPresentation::MeasurementPresentation(QWidget* view,vtkRenderer* renderer,ViewportResultOverlay* results)
    :renderer_(renderer),results_(results),legend_(std::make_unique<ViewportColorLegend>(view)),
     annotations_(std::make_unique<ViewportAnnotationOverlay>(view,renderer,results))
{
    annotations_->avoidWidget(legend_.get());
}
MeasurementPresentation::~MeasurementPresentation() { clear(); }
void MeasurementPresentation::addHeatmap(const QString& group,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,vtkActor* source)
{
    if (!cloud || cloud->empty()) return;
    auto points=vtkSmartPointer<vtkPoints>::New();auto cells=vtkSmartPointer<vtkCellArray>::New();
    auto colors=vtkSmartPointer<vtkUnsignedCharArray>::New();colors->SetNumberOfComponents(3);
    for (const auto& p:*cloud) {
        const auto id=points->InsertNextPoint(p.x,p.y,p.z);cells->InsertNextCell(1,&id);
        const unsigned char rgb[]={p.r,p.g,p.b};colors->InsertNextTypedTuple(rgb);
    }
    auto data=vtkSmartPointer<vtkPolyData>::New();data->SetPoints(points);data->SetVerts(cells);data->GetPointData()->SetScalars(colors);
    auto mapper=vtkSmartPointer<vtkPolyDataMapper>::New();mapper->SetInputData(data);
    auto actor=vtkSmartPointer<vtkActor>::New();actor->SetMapper(mapper);actor->PickableOff();actor->UseBoundsOff();
    actor->GetProperty()->SetPointSize(3);actor->GetProperty()->LightingOff();
    renderer_->AddActor(actor);groups_[group].heatmaps.push_back(actor);
    hideSource(group,source);
}
void MeasurementPresentation::hideSource(const QString& group,vtkActor* source)
{
    if (source) {
        auto found=sources_.find(source);
        if (found==sources_.end()) found=sources_.emplace(source,Source{source,source->GetVisibility(),{}}).first;
        found->second.owners.insert(group);source->VisibilityOff();
    }
}
void MeasurementPresentation::addCurve(const QString& group,vtkSmartPointer<vtkActor> actor,bool outline)
{
    if (!actor || !renderer_->GetRenderWindow()) return;
    auto window=renderer_->GetRenderWindow();
    if (!foreground_) {
        originalLayers_=window->GetNumberOfLayers();window->SetNumberOfLayers(originalLayers_+2);
        foregroundOutline_=vtkSmartPointer<vtkRenderer>::New();foreground_=vtkSmartPointer<vtkRenderer>::New();
        int layer=originalLayers_;
        for (auto renderer:{foregroundOutline_,foreground_}) {
            renderer->SetLayer(layer++);renderer->SetActiveCamera(renderer_->GetActiveCamera());renderer->InteractiveOff();
            renderer->SetPreserveDepthBuffer(0);renderer->SetPreserveColorBuffer(1);window->AddRenderer(renderer);
        }
    }
    actor->PickableOff();actor->UseBoundsOff();actor->GetProperty()->LightingOff();
    actor->GetProperty()->SetLineWidth(actor->GetProperty()->GetLineWidth()*legend_->parentWidget()->devicePixelRatioF());
    (outline ? foregroundOutline_:foreground_)->AddActor(actor);groups_[group].foreground.push_back(actor);
}
void MeasurementPresentation::releaseForeground()
{
    if (!foreground_) return;
    for (const auto& group:groups_) if (!group.second.foreground.empty()) return;
    auto window=renderer_->GetRenderWindow();window->RemoveRenderer(foreground_);window->RemoveRenderer(foregroundOutline_);foreground_=foregroundOutline_=nullptr;
    int layers=originalLayers_;auto renderers=window->GetRenderers();renderers->InitTraversal();
    while (auto renderer=renderers->GetNextItem()) layers=std::max(layers,renderer->GetLayer()+1);
    window->SetNumberOfLayers(layers);
}
void MeasurementPresentation::addGeometry(const QString& group,vtkSmartPointer<vtkActor> actor)
{
    if (!actor) return;
    actor->PickableOff();actor->UseBoundsOff();actor->GetProperty()->LightingOff();
    renderer_->AddActor(actor);groups_[group].geometry.push_back(actor);
}
void MeasurementPresentation::setAnnotations(const QString& group,std::vector<Annotation> annotations,bool append)
{
    auto& target=groups_[group].annotations;
    if (append) target.insert(target.end(),annotations.begin(),annotations.end());else target=std::move(annotations);
    updateAnnotations();
}
void MeasurementPresentation::appendResult(const QString& group,const QString& title,ViewportResultOverlay::Row row)
{
    auto& section=groups_[group].result;section.title=title;section.rows.push_back(std::move(row));results_->setSection(group,section);
}
void MeasurementPresentation::setLegend(const QString& group,ViewportColorLegend::Entry entry)
{
    auto& ids=groups_[group].legends;
    if (std::find(ids.begin(),ids.end(),entry.id)==ids.end()) ids.push_back(entry.id);
    legend_->setEntry(std::move(entry));
}
void MeasurementPresentation::softenSurfaces(const QString& group)
{
    auto actors=renderer_->GetActors();actors->InitTraversal();
    while (auto actor=actors->GetNextActor()) {
        if (!actor->GetMapper()) continue;
        actor->GetMapper()->Update();auto poly=vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
        if (!poly || !poly->GetNumberOfPolys()) continue;
        auto found=surfaces_.find(actor);
        if (found==surfaces_.end()) {
            auto property=vtkSmartPointer<vtkProperty>::New();property->DeepCopy(actor->GetProperty());
            found=surfaces_.emplace(actor,Surface{actor,property,{}}).first;
        }
        found->second.owners.insert(group);actor->GetProperty()->SetRepresentationToWireframe();actor->GetProperty()->SetOpacity(.18);
    }
}
void MeasurementPresentation::restoreContext(const QString& group)
{
    for (auto it=sources_.begin();it!=sources_.end();) {
        it->second.owners.erase(group);
        if (it->second.owners.empty()) { it->second.actor->SetVisibility(it->second.visibility);it=sources_.erase(it); } else ++it;
    }
    for (auto it=surfaces_.begin();it!=surfaces_.end();) {
        it->second.owners.erase(group);
        if (it->second.owners.empty()) { it->second.actor->GetProperty()->DeepCopy(it->second.property);it=surfaces_.erase(it); } else ++it;
    }
}
void MeasurementPresentation::updateAnnotations()
{
    std::vector<Annotation> all;
    for (const auto& group:groups_) all.insert(all.end(),group.second.annotations.begin(),group.second.annotations.end());
    annotations_->setAnnotations(std::move(all));
}
void MeasurementPresentation::clear(const QString& group)
{
    auto found=groups_.find(group);if (found==groups_.end()) return;
    for (auto& actor:found->second.heatmaps) renderer_->RemoveActor(actor);
    for (auto& actor:found->second.geometry) renderer_->RemoveActor(actor);
    for (auto& actor:found->second.foreground) if (foreground_) { foreground_->RemoveActor(actor);foregroundOutline_->RemoveActor(actor); }
    for (const auto& id:found->second.legends) legend_->removeEntry(id);
    results_->removeSection(group);restoreContext(group);groups_.erase(found);releaseForeground();updateAnnotations();
}
void MeasurementPresentation::clear() { while (!groups_.empty()) clear(groups_.begin()->first); }
void MeasurementPresentation::clearAnnotations()
{
    for (auto& group:groups_) { group.second.annotations.clear();group.second.legends.clear();group.second.result={};results_->removeSection(group.first); }
    annotations_->clear();legend_->clear();
}
void MeasurementPresentation::clearGeometry()
{
    for (auto& group:groups_) {
        for (auto& actor:group.second.geometry) renderer_->RemoveActor(actor);
        for (auto& actor:group.second.foreground) if (foreground_) { foreground_->RemoveActor(actor);foregroundOutline_->RemoveActor(actor); }
        group.second.foreground.clear();group.second.geometry.clear();group.second.annotations.clear();results_->removeSection(group.first);
    }
    annotations_->clear();releaseForeground();
}
void MeasurementPresentation::clearHeatmaps()
{
    std::vector<QString> ids;for (const auto& group:groups_) if (!group.second.heatmaps.empty()) ids.push_back(group.first);
    for (const auto& id:ids) clear(id);
}
