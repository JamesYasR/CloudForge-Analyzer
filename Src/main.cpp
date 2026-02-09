#include "CloudForgeAnalyzer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    CloudForgeAnalyzer win;
    win.show();
    return app.exec();
}
//#include "Basic/CloudClipper.h"
//#include "config/pcl114.h"
//int main() {
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile<pcl::PointXYZ>("PCDfiles/rabbit.pcd", *cloud);
//	interactivePolygonCut(cloud);
//}
