#include "Measure/MeasurePallel.h"

MeasurePallel::MeasurePallel(Eigen::Vector3f& v1, Eigen::Vector3f& v2)
{
	vec1 = v1;
	vec2 = v2;
	vec1.normalize();
	vec2.normalize();
}

float MeasurePallel::angleBetweenVectors() {
	float dot = vec1.dot(vec2);
	float norm = vec1.norm() * vec2.norm();
	float cosTheta = dot / norm;
	// ���������
	cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
	return std::acos(cosTheta) * 180.0f / M_PI;
}

float MeasurePallel::parallelism() {
	float theta = angleBetweenVectors();
	// ȡ��С�нǣ�0���180�㷽����ƽ�У�
	float minAngle = std::min(theta, 180.0f - theta);
	return minAngle;
}

float MeasurePallel::perpendicularity() {
	float theta = angleBetweenVectors();
	return std::abs(90.0f - theta);
}