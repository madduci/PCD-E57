//Last update: 10/10/2014

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//Contains the structure of XYZ point data
#include "e57/E57Foundation.h" //libE57 API
#include "e57/E57Simple.h"

#include "libconverter/e57.h"

#include "ArgParser.h"

using namespace e57;
using namespace converter;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PtrXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PtrXYZRGB;

[[deprecated]] int loadData(int argc, char **argv, std::vector<string> &files)
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
	app::ArgParser arg_parser{argc, argv};
	auto parameters = arg_parser.getParameters();

	std::cout << "Parameters size: " << parameters.size() << '\n';
	for (const auto &[key, value] : parameters)
	{
		std::cout << key << ": " << value << '\n';
	}

	std::vector<string> filenames;
	E57<PtrXYZI> e57;
	loadData(argc, argv, filenames);
	// Check user input
	if (filenames.empty())
	{
		std::cout << "Error checking files\n";
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
				std::cout << "Error reading file\n";
				return -1;
			}

			pcl::transformPointCloud(*cloud, *cloud_transformed, matrix);

			std::stringstream ss;
			ss << "Scan-" << i << "-" << scanIndex << ".pcd";
			std::cout << ss.str() << '\n';

			pcl::io::savePCDFileASCII(ss.str(), *cloud_transformed);

			//demonstration of writing down from PCD to E57
			if (e57.saveE57File("test.e57", cloud, scale_factor) == -1)
			{
				std::cout << "Error saving in e57\n";
				return -1;
			}

			std::cout << "********************* CONVERSION COMPLETED *********************\n";
			std::cout << "File: \t" << filenames.at(i) << '\n';
			std::cout << "Cloud Size: \t" << cloud.get()->size() << '\n';
			std::cout << "File Saved: \t" << ss.str() << '\n';
			std::cout << "********************* CONVERSION COMPLETED *********************\n";
		}
	}

	std::cout << "Conversion completed\n";
	return 0;
}
