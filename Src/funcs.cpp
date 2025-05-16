#include "funcs.h"

std::string to_string_with_precision(double num, int precision) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << num;
    return stream.str();
}

TimeStamp::TimeStamp() {}
TimeStamp::~TimeStamp()=default;

void TimeStamp::Start() {
    start_time = std::chrono::system_clock::now();
}

void TimeStamp::End() {
    end_time = std::chrono::system_clock::now();
}

std::string TimeStamp::get_QString_time_duration() {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    return std::to_string(duration) + "ms";
}

vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients& coefficients, double x, double y, double z, float scale[2] )
{
    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();


    double norm_sqr = 1.0 / (coefficients.values[0] * coefficients.values[0] +
        coefficients.values[1] * coefficients.values[1] +
        coefficients.values[2] * coefficients.values[2]);

    plane->SetNormal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    double t = x * coefficients.values[0] + y * coefficients.values[1] + z * coefficients.values[2] + coefficients.values[3];
    x -= coefficients.values[0] * t * norm_sqr;
    y -= coefficients.values[1] * t * norm_sqr;
    z -= coefficients.values[2] * t * norm_sqr;

    plane->SetCenter(x, y, z);

    {
        double pt1[3], pt2[3], orig[3], center[3];
        plane->GetPoint1(pt1);
        plane->GetPoint2(pt2);
        plane->GetOrigin(orig);
        plane->GetCenter(center);

        float scale1 = 3.0;
        float scale2 = 3.0;
        if (scale != nullptr)
        {
            scale1 = scale[0];
            scale2 = scale[1];
        }
        // ӳ pt1,pt2.  ӳ origin  pt1   ߵķ       
        double _pt1[3], _pt2[3];
        for (int i = 0; i < 3; i++) {
            _pt1[i] = scale1 * (pt1[i] - orig[i]);
            _pt2[i] = scale2 * (pt2[i] - orig[i]);
        }
        //pt1     ԭ    ϵ µ     ֵ
        for (int i = 0; i < 3; ++i)
        {
            pt1[i] = orig[i] + _pt1[i];
            pt2[i] = orig[i] + _pt2[i];
        }
        plane->SetPoint1(pt1);
        plane->SetPoint2(pt2);


        //        // ӳ origin
        //        double _origin[3];
        //        for(int i=0; i<3;++i)
        //        {
        //            _origin[i] = scale*(orig[i]-pt1[i]);
        //        }
        //        for(int i=0; i<3;++i)
        //        {
        //            orig[i] = pt1[i] + _origin[i];
        //        }
        //        plane->SetOrigin(orig);
    }
    plane->Update();

    return (plane->GetOutput());
}
