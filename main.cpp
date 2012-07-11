//Last update: 11/07/2012

#include "e57.h"
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace e57;
	
typedef pcl::PointXYZI P_XYZI;	
typedef pcl::PointCloud<P_XYZI>::Ptr PtrXYZI;

int loadData (int argc, char **argv, vector<string> &files)
{
	std::string extension (".e57");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string (argv[i]);
		cout << fname <<endl;
		files.push_back(fname);	 
		return 1;
	}
}



int main (int argc, char** argv){
	
	vector<string> filenames;
	E57 i_o;
	
	loadData (argc, argv, filenames);
	// Check user input
	if (filenames.empty ())
	{
		cout << "Error checking files"<<endl;
		return (-1);
	}
 
	PtrXYZI cloud(new pcl::PointCloud<P_XYZI>);
	PtrXYZI cloud_scaled(new pcl::PointCloud<P_XYZI>);
	
	float scale_factor = 0;
	float leaf_size = 150;
	for(int i = 0; i < filenames.size(); i++){
		if(!i_o.openE57(filenames.at(i), cloud, scale_factor)){
			cout << "Error reading file"<<endl;
			return -1;
		}
		
		pcl::VoxelGrid<P_XYZI> voxel;
		voxel.setInputCloud(cloud);
		leaf_size = leaf_size * scale_factor; //adapting by scale_factor
		voxel.setLeafSize (leaf_size, leaf_size, leaf_size); 		//filter in all the 3 directions (X, Y, Z)
		voxel.filter(*cloud_scaled);
		std::stringstream ss;
		ss <<"Cloud_"<< i << ".pcd";
		cout << ss.str()<<endl;
		pcl::io::savePCDFileASCII (ss.str(), *cloud_scaled);
		//demonstration of writing down from PCD to E57
		if(i_o.saveE57File("prova.e57", cloud_scaled, scale_factor) == 0){
			cout << "Error saving in e57"<<endl;
			return -1;
		}
		cout << "********************* SCALING COMPLETED *********************"<<endl;
		cout << "File: "<<filenames.at(i)<<endl;
		cout << "Before Scaling: "<<cloud.get()->size()<<endl;
		cout << "Scaled: "<<cloud_scaled.get()->size()<<" | with scale factor: "<<scale_factor<<" | with leafsize: "<<leaf_size<<endl;
		cout << "File Saved: "<< ss.str() <<endl;
		cout << "********************* SCALING COMPLETED *********************"<<endl;
	}
	
	cout << "Downscaling completed" << endl;
	return 0;
}
