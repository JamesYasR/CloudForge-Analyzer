#ifndef POLYGONCUTTOOL_H
#define POLYGONCUTTOOL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkMath.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <iostream>
#include <vector>
#include <map>

// 前置声明
namespace pcl {
    template <typename PointT>
    class PointCloud;
}

// 点云类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 函数声明
void interactivePolygonCut(PointCloudT::Ptr cloud);

#endif // POLYGONCUTTOOL_H