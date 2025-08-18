#include "CloudForgeAnalyzer.h"
#include <QtWidgets/QApplication>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    CloudForgeAnalyzer win;
    win.show();
    return app.exec();
}
