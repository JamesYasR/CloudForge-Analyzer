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
	// 处理浮点误差
	cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
	return std::acos(cosTheta) * 180.0f / M_PI;
}

float MeasurePallel::parallelism() {
	float theta = angleBetweenVectors();
	// 取最小夹角（0°或180°方向都算平行）
	float minAngle = std::min(theta, 180.0f - theta);
	return minAngle;
}

float MeasurePallel::perpendicularity() {
	float theta = angleBetweenVectors();
	return std::abs(90.0f - theta);
}