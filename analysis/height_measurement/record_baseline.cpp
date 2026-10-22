#include "cases.h"
#include <iostream>
int main()
{
    for (int i=0;i<5;++i) {
        auto item=heightCase(i);
        MeasureHeight measurement(item.measured,item.reference);
        std::vector<double> averages;
        if (!measurement.measureIterative(averages,10,5)) return 1;
        std::cout<<heightSnapshot(measurement,averages,originalHeightColors(item,measurement));
    }
}
