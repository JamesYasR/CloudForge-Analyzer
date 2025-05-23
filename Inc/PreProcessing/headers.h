#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <QLineEdit>
#include <QIntValidator>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include "Dialog/Dialog.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include <map>
#include <pcl/visualization/cloud_viewer.h>
#include <QtWidgets/QMainWindow>
#include "CloudForgeAnalyzer.h"
#include <qstring.h>
#include <qdebug.h>