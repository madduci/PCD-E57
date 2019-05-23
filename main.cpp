//Last update: 10/10/2014

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//Contains the structure of XYZ point data
#include "E57/E57Foundation.h" //libE57 API
#include "E57/E57Simple.h"

#include "e57.h"

using namespace std;
using namespace e57;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PtrXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtrXYZRGB;

int loadData(int argc, char **argv, vector<string> &files)
{
	std::string extension(".e57");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; ++i)
	{
		std::string fname = std::string(argv[i]);
		cout << fname << endl;
		files.push_back(fname);
		return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	vector<string> filenames;
	E57<PtrXYZI> e57;
	loadData(argc, argv, filenames);
	// Check user input
	if (filenames.empty())
	{
		cout << "Error checking files" << endl;
		return (-1);
	}

	PtrXYZI cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	PtrXYZI cloud_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

	float scale_factor = 0.0f;
	for (auto i = 0; i < filenames.size(); ++i)
	{
		// Will be overwritten:
		int64_t scanCount = 1;

		for (int64_t scanIndex = 0; scanIndex < scanCount; ++scanIndex)
		{
			Eigen::Matrix4f matrix;
			if (e57.openE57(filenames.at(i), cloud, scale_factor, scanCount, matrix, scanIndex) == -1)
			{
				cout << "Error reading file" << endl;
				return -1;
			}

			pcl::transformPointCloud(*cloud, *cloud_transformed, matrix);

			std::stringstream ss;
			ss << "Scan-" << i << "-" << scanIndex << ".pcd";
			cout << ss.str() << endl;

			pcl::io::savePCDFileASCII(ss.str(), *cloud_transformed);

			//demonstration of writing down from PCD to E57
			if (e57.saveE57File("test.e57", cloud, scale_factor) == -1)
			{
				cout << "Error saving in e57" << endl;
				return -1;
			}

			cout << "********************* CONVERSION COMPLETED *********************" << endl;
			cout << "File: \t" << filenames.at(i) << endl;
			cout << "Cloud Size: \t" << cloud.get()->size() << endl;
			cout << "File Saved: \t" << ss.str() << endl;
			cout << "********************* CONVERSION COMPLETED *********************" << endl;
		}
	}

	cout << "Conversion completed" << endl;
	return 0;
}
