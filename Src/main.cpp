#include "CloudForgeAnalyzer.h"
#include <QtWidgets/QApplication>
#include "Loops.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    CloudForgeAnalyzer win;

    Count_Points cp(win.cloud,win.ui);//循环事件

    win.show();
    return app.exec();
}
