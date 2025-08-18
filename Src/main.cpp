#include "CloudForgeAnalyzer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    CloudForgeAnalyzer win;
    win.show();
    return app.exec();
}
