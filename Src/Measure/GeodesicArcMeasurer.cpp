#include "Measure/GeodesicArcMeasurer.h"
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyLine.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkTextActor.h>
#include <vtkProperty.h>
#include <vtkTriangle.h>
#include <vtkAssembly.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <limits>
#include <cmath>

using namespace pcl;

GeodesicArcMeasurer::GeodesicArcMeasurer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double search_radius)
    : input_cloud_(cloud)
    , base_radius_(search_radius),
    curvature_scale(0.6),
    geodesic_scale(1.2),
    curvature_radius_(search_radius * 0.6),   // 默认比例
    geodesic_radius_(search_radius * 1.2)     // 默认比例
{
    buildSurfaceModel(search_radius);
}



void GeodesicArcMeasurer::buildSurfaceModel(double search_radius)
{
    // 使用MLS进行曲面重建和平滑
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    mls.setInputCloud(input_cloud_);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);
    mls.setPolynomialOrder(2); // 二次多项式拟合
    mls.setComputeNormals(true);

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    mls.process(*mls_points);

    // 提取曲面点和法向量
    surface_points_.resize(mls_points->size());
    for (size_t i = 0; i < mls_points->size(); ++i) {
        surface_points_[i].position = mls_points->points[i].getVector3fMap();
        surface_points_[i].normal = mls_points->points[i].getNormalVector3fMap();
    }
    kdtree_.setInputCloud(input_cloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : *mls_points) {
        mls_cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    surface_kdtree_.setInputCloud(mls_cloud); // 初始化 surface_kdtree_
    // 计算曲率
    computeCurvature();

    // 构建KD树用于快速最近邻搜索
   
}

void GeodesicArcMeasurer::computeCurvature()
{
    // 使用 MLS 点云的专用 KD 树进行搜索
    for (size_t i = 0; i < surface_points_.size(); ++i) {
        const auto& point = surface_points_[i].position;
        pcl::PointXYZ search_point(point.x(), point.y(), point.z());

        // 使用 MLS 点云的 KD 树查找邻近点
        std::vector<int> point_idx;
        std::vector<float> point_squared_distance;
        surface_kdtree_.radiusSearch(search_point, curvature_radius_, point_idx, point_squared_distance);

        // 检查邻近点数量是否足够计算曲率
        if (point_idx.size() < 3) {
            surface_points_[i].curvature = 0.0f;
            continue;
        }

        // 计算邻近点的质心
        Eigen::Vector3f centroid(0, 0, 0);
        for (int idx : point_idx) {
            // 使用 MLS 点云的索引安全访问
            centroid += surface_points_[idx].position;
        }
        centroid /= static_cast<float>(point_idx.size());

        // 计算协方差矩阵
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (int idx : point_idx) {
            Eigen::Vector3f d = surface_points_[idx].position - centroid;
            covariance += d * d.transpose();
        }
        covariance /= static_cast<float>(point_idx.size());

        // 计算特征值和曲率
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f eigenvalues = solver.eigenvalues();

        // 曲率计算：最小特征值 / 特征值之和
        surface_points_[i].curvature = eigenvalues(0) / (eigenvalues(0) + eigenvalues(1) + eigenvalues(2));
    }
}

int GeodesicArcMeasurer::findNearestSurfacePoint(const pcl::PointXYZ& point)
{
    std::vector<int> point_idx(1);
    std::vector<float> point_squared_distance(1);

    if (surface_kdtree_.nearestKSearch(point, 1, point_idx, point_squared_distance) > 0) {
        return point_idx[0];
    }
    return -1;
}

double GeodesicArcMeasurer::computeGeodesicDistance(int start_idx, int end_idx, std::vector<Eigen::Vector3f>& path_points)
{
    if (start_idx == end_idx) {
        path_points.push_back(surface_points_[start_idx].position);
        return 0.0;
    }

    // 初始化距离数组
    std::vector<double> distances(surface_points_.size(), std::numeric_limits<double>::max());
    std::vector<int> predecessors(surface_points_.size(), -1);
    distances[start_idx] = 0.0;

    // 优先队列用于Dijkstra算法
    auto compare = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second > b.second;
        };
    std::priority_queue<std::pair<int, double>,
        std::vector<std::pair<int, double>>,
        decltype(compare)> pq(compare);
    pq.push({ start_idx, 0.0 });

    while (!pq.empty()) {
        auto [current_idx, current_dist] = pq.top();
        pq.pop();

        if (current_idx == end_idx) {
            // 回溯构建路径
            std::vector<int> path_indices;
            for (int at = end_idx; at != -1; at = predecessors[at]) {
                path_indices.push_back(at);
            }
            std::reverse(path_indices.begin(), path_indices.end());

            // 提取路径点
            for (int idx : path_indices) {
                path_points.push_back(surface_points_[idx].position);
            }

            return current_dist;
        }

        if (current_dist > distances[current_idx]) {
            continue;
        }

        // 获取当前点的邻近点
        pcl::PointXYZ search_point(
            surface_points_[current_idx].position.x(),
            surface_points_[current_idx].position.y(),
            surface_points_[current_idx].position.z());

        std::vector<int> neighbor_indices;
        std::vector<float> squared_distances;
        kdtree_.radiusSearch(search_point, geodesic_radius_, neighbor_indices, squared_distances);

        for (int neighbor_idx : neighbor_indices) {
            if (neighbor_idx == current_idx) continue;

            // 计算测地距离近似值
            double edge_length = (surface_points_[current_idx].position -
                surface_points_[neighbor_idx].position).norm();

            // 考虑曲率影响（曲率越大，距离权重越大）
            double curvature_weight = 1.0 + 10.0 * surface_points_[current_idx].curvature;
            double new_dist = current_dist + edge_length * curvature_weight;

            if (new_dist < distances[neighbor_idx]) {
                distances[neighbor_idx] = new_dist;
                predecessors[neighbor_idx] = current_idx;
                pq.push({ neighbor_idx, new_dist });
            }
        }
    }

    return -1.0; // 未找到路径
}

GeodesicArcMeasurer::MeasurementResult GeodesicArcMeasurer::measureArcLength(const pcl::PointXYZ& start, const pcl::PointXYZ& end)
{
    MeasurementResult result;

    // 找到曲面上最近的点
    int start_idx = findNearestSurfacePoint(start);
    int end_idx = findNearestSurfacePoint(end);

    if (start_idx == -1 || end_idx == -1) {
        std::cerr << "Error: Could not find start or end point on surface." << std::endl;
        return result;
    }

    // 使用Dijkstra算法计算测地距离
    result.arc_length = computeGeodesicDistance(start_idx, end_idx, result.path_points);
    result.success = (result.arc_length >= 0);

    return result;
}

vtkSmartPointer<vtkActor> GeodesicArcMeasurer::createSurfaceActor() {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();

    // 添加所有曲面点到VTK点集
    for (const auto& sp : surface_points_) {
        points->InsertNextPoint(sp.position.x(), sp.position.y(), sp.position.z());
    }

    // 创建曲面网格（三角化）
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud_);

    const double radius = 0.05; // 搜索半径
    for (size_t i = 0; i < surface_points_.size(); i += 5) { // 采样减少点数量
        const auto& point = surface_points_[i].position;
        pcl::PointXYZ search_point(point.x(), point.y(), point.z());

        std::vector<int> point_idx;
        std::vector<float> point_squared_distance;
        kdtree.radiusSearch(search_point, radius, point_idx, point_squared_distance);

        if (point_idx.size() >= 3) {
            // 创建三角形
            for (size_t j = 1; j < point_idx.size() - 1; j++) {
                vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
                triangle->GetPointIds()->SetId(0, i);
                triangle->GetPointIds()->SetId(1, point_idx[j]);
                triangle->GetPointIds()->SetId(2, point_idx[j + 1]);
                cells->InsertNextCell(triangle);
            }
        }
    }

    // 创建多边形数据
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(cells);

    // 创建映射器和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.7, 0.7, 0.9); // 浅蓝色
    actor->GetProperty()->SetOpacity(0.5); // 半透明
    actor->GetProperty()->SetLighting(true);
    actor->GetProperty()->SetInterpolationToGouraud();

    return actor;
}

vtkSmartPointer<vtkActor> GeodesicArcMeasurer::createPointCloudActor() {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& pt : *input_cloud_) {
        points->InsertNextPoint(pt.x, pt.y, pt.z);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.8, 0.8, 0.8); // 浅灰色
    actor->GetProperty()->SetPointSize(2);
    actor->GetProperty()->SetLighting(false); // 点云通常不需要光照

    return actor;
}

vtkSmartPointer<vtkActor> GeodesicArcMeasurer::createPathActor(const std::vector<Eigen::Vector3f>& path_points) {
    if (path_points.size() < 2) {
        return nullptr; // 路径点不足，无法创建路径
    }

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    // 添加路径点
    for (const auto& pt : path_points) {
        points->InsertNextPoint(pt.x(), pt.y(), pt.z());
    }

    // 创建线
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(path_points.size());
    for (size_t i = 0; i < path_points.size(); ++i) {
        polyLine->GetPointIds()->SetId(i, i);
    }
    lines->InsertNextCell(polyLine);

    // 创建多边形数据
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建映射器和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 橙色
    actor->GetProperty()->SetLineWidth(3);
    actor->GetProperty()->SetLighting(false); // 路径线通常不需要光照

    return actor;
}

vtkSmartPointer<vtkAssembly> GeodesicArcMeasurer::createStartEndActors(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end)
{
    // 创建起点球体
    vtkSmartPointer<vtkSphereSource> startSphere = vtkSmartPointer<vtkSphereSource>::New();
    startSphere->SetCenter(start.x(), start.y(), start.z());
    startSphere->SetRadius(0.005);
    startSphere->SetPhiResolution(20);
    startSphere->SetThetaResolution(20);

    vtkSmartPointer<vtkPolyDataMapper> startMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    startMapper->SetInputConnection(startSphere->GetOutputPort());

    vtkSmartPointer<vtkActor> startActor = vtkSmartPointer<vtkActor>::New();
    startActor->SetMapper(startMapper);
    startActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色

    // 创建终点球体
    vtkSmartPointer<vtkSphereSource> endSphere = vtkSmartPointer<vtkSphereSource>::New();
    endSphere->SetCenter(end.x(), end.y(), end.z());
    endSphere->SetRadius(0.005);
    endSphere->SetPhiResolution(20);
    endSphere->SetThetaResolution(20);

    vtkSmartPointer<vtkPolyDataMapper> endMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    endMapper->SetInputConnection(endSphere->GetOutputPort());

    vtkSmartPointer<vtkActor> endActor = vtkSmartPointer<vtkActor>::New();
    endActor->SetMapper(endMapper);
    endActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色

    // 合并两个actor
    vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();
    assembly->AddPart(startActor);
    assembly->AddPart(endActor);

    return assembly;
}

vtkSmartPointer<vtkTextActor> GeodesicArcMeasurer::createTextActor(double arc_length) {
    // 创建文本Actor
    vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();

    // 设置文本内容
    std::stringstream ss;
    ss << "Arc Length: " << std::fixed << std::setprecision(3) << arc_length << " units";
    textActor->SetInput(ss.str().c_str());

    // 设置位置
    textActor->SetPosition(20, 30);

    // 获取并设置文本属性
    vtkTextProperty* textProp = textActor->GetTextProperty();
    if (textProp) {
        textProp->SetFontSize(18);
        textProp->SetColor(1.0, 1.0, 0.0); // 黄色
        textProp->SetBackgroundColor(0.0, 0.0, 0.0);
        textProp->SetBackgroundOpacity(0.5);
        textProp->SetBold(true);
        textProp->SetShadow(true);
        textProp->SetFontFamilyToArial();
        textProp->SetJustificationToLeft();
        textProp->SetVerticalJustificationToBottom();
    }

    return textActor;
}

// 修改createVisualizationActors返回类型为vtkProp
std::pair<std::vector<vtkSmartPointer<vtkProp>>,
    std::vector<vtkSmartPointer<vtkTextActor>>>
    GeodesicArcMeasurer::createVisualizationActors(
        const MeasurementResult& result,
        bool show_surface,
        bool show_path,
        bool show_points,
        bool show_legend)
{
    std::vector<vtkSmartPointer<vtkProp>> actors_3d;
    std::vector<vtkSmartPointer<vtkTextActor>> actors_2d;

    if (show_surface) {
        auto surfaceActor = createSurfaceActor();
        if (surfaceActor) actors_3d.push_back(surfaceActor);
    }

    if (show_points) {
        auto pointCloudActor = createPointCloudActor();
        if (pointCloudActor) actors_3d.push_back(pointCloudActor);
    }

    if (show_path && result.success && result.path_points.size() >= 2) {
        auto pathActor = createPathActor(result.path_points);
        if (pathActor) actors_3d.push_back(pathActor);

        auto startEndActors = createStartEndActors(
            result.path_points.front(),
            result.path_points.back());
        actors_3d.push_back(startEndActors);
    }

    if (show_legend && result.success) {
        auto textActor = createTextActor(result.arc_length);
        actors_2d.push_back(textActor);
    }

    return std::make_pair(actors_3d, actors_2d);
}

void GeodesicArcMeasurer::setRadiusScaleFactors(double curvature_scale, double geodesic_scale) {
    curvature_radius_ = base_radius_ * curvature_scale;
    geodesic_radius_ = base_radius_ * geodesic_scale;
}

// 直接设置绝对值（自动计算比例）
void GeodesicArcMeasurer::setCurvatureRadius(double radius) {
    curvature_radius_ = radius;
    // 存储比例用于后续重建
	curvature_scale = radius / base_radius_;
}

void GeodesicArcMeasurer::setGeodesicRadius(double radius) {
    geodesic_radius_= radius;
    // 存储比例用于后续重建
    geodesic_scale = radius / base_radius_;
}