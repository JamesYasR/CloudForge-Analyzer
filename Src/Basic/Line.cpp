#include "Basic/Line.h"
Line::Line(pcl::PointXYZ startp, pcl::PointXYZ endp, ColorManager colm, double wid, Eigen::VectorXf coef) :
	color(ColorManager(255,0,0))
{
	start = startp;
	end = endp;
	color = colm;
	width = wid;
	coeffs = coef;
	dir_vector = Eigen::Vector3f(coeffs[3], coeffs[4], coeffs[5]);
}
Line::Line() :
	color(ColorManager(255, 0, 0))
{

}