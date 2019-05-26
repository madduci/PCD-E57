/*
************************ Last revision of this file ***********************
* $Author:: Michele Adduci
* $LastChangedDate:: 23/05/2019
**************************************************************************
*/

#ifndef E57_H
#define E57_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

struct StructMaxMin
{
	float max;
	float min;
};

template <typename PointCloud>
class E57
{

private:
	//Used for discovering Max and Minimum of a Pointcloud, before saving it in E57: this is an info required by E57 file format
	void findMaxMin(PointCloud &pointcloud, StructMaxMin *vector);

public:
	int openE57(const std::string &filename, PointCloud &pointcloud, float &scale_factor, int64_t &scanCount, Eigen::Matrix4f &mat4, int64_t scanIndex = 0);

	int saveE57File(const std::string &filename, PointCloud &cloud, float &scale_factor, int index = 0);
};

#endif
