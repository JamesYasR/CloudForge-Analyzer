#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/project_inliers.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace pcl;

class MeasureArc {
public:
	MeasureArc();
	~MeasureArc();
	void FitCylinder(PointCloud<PointXYZ>::Ptr inputcloud);
	double calculateArcLength();
private:
	
	PointCloud<PointXYZ>::Ptr cloud;
	Eigen::Vector3f& center;
	Eigen::Vector3f& axis_direction;
	double radius;
};