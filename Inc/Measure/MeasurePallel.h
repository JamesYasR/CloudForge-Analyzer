#pragma once
#include "config/pcl114.h"
class MeasurePallel
{
public:
	MeasurePallel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);
	~MeasurePallel()=default;
	float parallelism();
	float perpendicularity();
private:
	Eigen::Vector3f vec1;
	Eigen::Vector3f vec2;
	float angleBetweenVectors();
};
