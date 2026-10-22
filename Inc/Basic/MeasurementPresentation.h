#pragma once
#include "Basic/ViewportAnnotationOverlay.h"
#include "Basic/ViewportColorLegend.h"
#include "Basic/ViewportResultOverlay.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <memory>
#include <set>

class vtkActor;
class vtkProperty;
class MeasurementPresentation
{
public:
    using Annotation=ViewportAnnotationOverlay::Annotation;
    MeasurementPresentation(QWidget* view,vtkRenderer* renderer,ViewportResultOverlay* results);
    ~MeasurementPresentation();
    void addHeatmap(const QString& group,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,vtkActor* source=nullptr);
    void addGeometry(const QString& group,vtkSmartPointer<vtkActor> actor);
    void setAnnotations(const QString& group,std::vector<Annotation> annotations,bool append=false);
    void appendResult(const QString& group,const QString& title,ViewportResultOverlay::Row row);
    void setLegend(const QString& group,ViewportColorLegend::Entry entry);
    void softenSurfaces(const QString& group);
    void clear(const QString& group);
    void clear();
    void clearAnnotations();
    void clearGeometry();
    void clearHeatmaps();
private:
    struct Group {
        std::vector<vtkSmartPointer<vtkActor>> heatmaps,geometry;
        std::vector<Annotation> annotations;
        std::vector<QString> legends;
        ViewportResultOverlay::Section result;
    };
    struct Source { vtkSmartPointer<vtkActor> actor;int visibility=1;std::set<QString> owners; };
    struct Surface { vtkSmartPointer<vtkActor> actor;vtkSmartPointer<vtkProperty> property;std::set<QString> owners; };
    void updateAnnotations();
    void restoreContext(const QString& group);
    vtkRenderer* renderer_;
    ViewportResultOverlay* results_;
    std::unique_ptr<ViewportColorLegend> legend_;
    std::unique_ptr<ViewportAnnotationOverlay> annotations_;
    std::map<QString,Group> groups_;
    std::map<vtkActor*,Source> sources_;
    std::map<vtkActor*,Surface> surfaces_;
};
