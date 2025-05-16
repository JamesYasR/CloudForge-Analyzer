#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QMap>
#include <QDialog>
#include <pcl/point_cloud.h>
#include <QScrollArea>
#include <QCheckBox>
#include <QColor>
#include <QMessageBox>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <QDIR>
#include <QFILEDIALOG>
#include <pcl/common/transforms.h>  // 变换操作
#include <vtkMatrix4x4.h>           // VTK矩阵
#include <pcl/point_types_conversion.h> // 点云转换
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include "ColorManager.h"
#include "Dialog/Dialog_SelectPointCloud.h"


