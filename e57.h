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
#include <iostream>

#include <string>
//Contains the structure of XYZ point data
#include "E57/E57Foundation.h" //libE57 API

typedef pcl::PointXYZI P_XYZ;
typedef pcl::PointCloud<P_XYZ>::Ptr PtrXYZ;

struct StructMaxMin
{
	float max;
	float min;
};

class E57
{

private:
	//Used for discovering Max and Minimum of a Pointcloud, before saving it in E57: this is an info required by E57 file format
	void findMaxMin(PtrXYZ &pointcloud, StructMaxMin *vector);

public:
	E57() {}
	~E57() {}

	int openE57(const std::string &filename, PtrXYZ &pointcloud, float &scale_factor, int64_t &scanCount, Eigen::Matrix4f &mat4, int64_t scanIndex = 0);

	int saveE57File(const std::string &filename, PtrXYZ &cloud, float &scale_factor, int index = 0);
};

#endif
