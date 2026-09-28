#include "Basic/Line.h"
Line::Line(pcl::PointXYZ startp, pcl::PointXYZ endp, ColorManager colm, double wid, Eigen::VectorXf coef) :
	color(ColorManager(255,0,0))
{
	start = startp;
	end = endp;
	color = colm;
	width = wid;
	coeffs = coef;
	// coeffs 允许为空: 多数调用只给两端点(AddLine 的第 6 参数有默认值)。
	// 原实现在空 coeffs 上直接取 coeffs[3..5], 会触发 Eigen 断言 "index < size()" 直接崩溃。
	if (coeffs.size() >= 6) {
		dir_vector = Eigen::Vector3f(coeffs[3], coeffs[4], coeffs[5]);
	}
	else {
		// 退化为用两端点之差作方向(单位化; 两端点重合时保持零向量)
		dir_vector = Eigen::Vector3f(endp.x - startp.x, endp.y - startp.y, endp.z - startp.z);
		const float n = dir_vector.norm();
		if (n > 1e-12f) dir_vector /= n;
	}
}
Line::Line() :
	color(ColorManager(255, 0, 0))
{
	dir_vector = Eigen::Vector3f::Zero();   // 避免未初始化被读取
}