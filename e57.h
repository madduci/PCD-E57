//*********************** Last revision of this file ***********************
//$Author:: Michele Adduci
//$LastChangedDate:: 10/10/2014
//**************************************************************************
//

#ifndef E57_H
#define E57_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <ctime>
#include <string>
//Contains the structure of XYZ point data
#include "E57/E57Foundation.h"  //libE57 API
#include "E57/E57Simple.h"

using namespace std;
using namespace e57;

typedef pcl::PointXYZI P_XYZ;
typedef pcl::PointCloud<P_XYZ>::Ptr PtrXYZ;

struct StructMaxMin{
	float max;
	float min;
};

class E57{
	
	private:
		//Used for discovering Max and Minimum of a Pointcloud, before saving it in E57: this is an info required by E57 file format
        inline void findMaxMin(PtrXYZ &pointcloud, StructMaxMin *vector){

			//setup of max and min for the 4 values of PointCloud structure
			vector[0].max = pointcloud->points[0].x;
			vector[0].min = pointcloud->points[0].x;
			vector[1].max = pointcloud->points[0].y;
			vector[1].min = pointcloud->points[0].y;
			vector[2].max = pointcloud->points[0].z;
            vector[2].min = pointcloud->points[0].z;
			
            for(auto &point:pointcloud->points)
            {
                /* Block for X coordinate */
                if(vector[0].max < point.x)
                {
                    vector[0].max = point.x;
                }
                else if(vector[0].min > point.x)
                {
                    vector[0].min = point.x;
                }

                /* Block for Y coordinate */
                if(vector[1].max < point.y)
                {
                    vector[1].max = point.y;
                }
                else if(vector[1].min > point.y)
                {
                    vector[1].min = point.y;
                }

                /* Block for Z coordinate */
                if(vector[2].max < point.z)
                {
                    vector[2].max = point.z;
                }
                else if(vector[2].min > point.z)
                {
                    vector[2].min = point.z;
                }
            }
            cout<<"Values found: "<<endl;
            for(int i=0; i<3; ++i)
            {
                cout << i << "Max Value: " << vector[i].max << " Min Value: " << vector[i].min << endl;
            }
		}

	public:
        E57(){}
        ~E57(){}
        inline int openE57(const std::string &filename, PtrXYZ &pointcloud, float &scale_factor, int64_t& scanCount, Eigen::Matrix4f& mat4, int64_t scanIndex = 0){
			try{
				ImageFile imf(filename, "r");
			    StructureNode root = imf.root();
				/// Make sure vector of scans is defined and of expected type.
				/// If "/data3D" wasn't defined, the call to root.get below would raise an exception.
				if (!root.isDefined("/data3D")) {
					cout << "File doesn't contain 3D images."<<endl;
					return 0;
				}
				Node n = root.get("/data3D");
				if (n.type() != E57_VECTOR) {
					cout <<"File Corrupted. Error during opening."<<endl;
					return 0;
				}
				
				/// The node is a vector so we can safely get a VectorNode handle to it.
				/// If n was not a VectorNode, this would raise an exception.
				VectorNode data3D(n);
			
				/// Print number of children of data3D.  This is the number of scans in file.
				scanCount = data3D.childCount();
				//~ 
				
				if(scanCount == 0 || scanIndex < 0 || scanIndex >= scanCount){
					cout <<"File doesn't contain valid informations."<<endl;
					return 0;
				}
				
				/// Get scan from "/data3D", assume its a Structure (else get exception)
				StructureNode scan(data3D.get(scanIndex));
				std::cout << scan.elementName() << std::endl;

				StructureNode pose(scan.get("pose"));
				StructureNode rotation(pose.get("rotation"));
				StructureNode translation(pose.get("translation"));
				float rx = FloatNode(rotation.get("x")).value();
				float ry = FloatNode(rotation.get("y")).value();
				float rz = FloatNode(rotation.get("z")).value();
				float rw = FloatNode(rotation.get("w")).value();
				float tx = FloatNode(translation.get("x")).value();
				float ty = FloatNode(translation.get("y")).value();
				float tz = FloatNode(translation.get("z")).value();
				std::cout << tx << " " << ty << " " << tz << " " << rx << " " << ry << " " << rz << " " << rw << std::endl;

				Eigen::Matrix3f mat3 = Eigen::Quaternionf(rw, rx, ry, rz).toRotationMatrix();
				mat4 = Eigen::Matrix4f::Identity();
				mat4.block(0,0,3,3) = mat3;
				mat4.block(0,3,3,1) = Eigen::Vector3f(tx, ty, tz);

				/// Get "points" field in scan.  Should be a CompressedVectorNode.
				CompressedVectorNode points(scan.get("points"));
				
				cout<<"Points: "<<points.childCount()<<endl;
				pointcloud->width = points.childCount();
				pointcloud->height = 1;
				pointcloud->is_dense = false;
				pointcloud->resize(pointcloud->width * pointcloud->height);
				/// Call subroutine in this file to print the points
				StructureNode proto(points.prototype());
			    /// The prototype should have a field named either "cartesianX" or "sphericalRange".
                if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ")) {
			        /// Make a list of buffers to receive the xyz values.
			        vector<SourceDestBuffer> destBuffers;
					float *x = new float[points.childCount()];
					float *y = new float[points.childCount()];
                    float *z = new float[points.childCount()];
                    destBuffers.push_back(SourceDestBuffer(imf, "cartesianX", x, points.childCount(), true));
			        destBuffers.push_back(SourceDestBuffer(imf, "cartesianY", y, points.childCount(), true));
			        destBuffers.push_back(SourceDestBuffer(imf, "cartesianZ", z, points.childCount(), true));
                    //destBuffers.push_back(SourceDestBuffer(imf, "intensity", intensity, points.childCount(), true));
			        /// Create a reader of the points CompressedVector, try to read first block of N points
			        /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
                    CompressedVectorReader reader = points.reader(destBuffers);
                    if(reader.read() <= 0)
                    {
                        cout << "Failed to read E57 file" << endl;
                        return -1;
                    }
			        float min_scale = 100;	//assigned an high value before starting.
                    auto j = 0;
                    for(auto &point:pointcloud->points){
						
                        point.x = x[j];	//seems E57 is expressed in millimeters
                        point.y = y[j];	//seems E57 is expressed in millimeters
                        point.z = z[j];	//seems E57 is expressed in millimeters
						
                        if(point.x > 10000 || point.y > 10000 || point.z > 10000)
                        {
                            point.x = point.x  * 0.001;
                            point.y = point.y  * 0.001;
                            point.z = point.z  * 0.001;
                            if(min_scale > 0.001)
                                min_scale = 0.001;
						}
                        if(point.x > 1000 || point.y > 1000 || point.z > 1000){
                            point.x = point.x  * 0.01;
                            point.y = point.y  * 0.01;
                            point.z = point.z  * 0.01;
							if(min_scale > 0.01)
								min_scale = 0.01;
						}
                        else if(point.x > 100 || point.y > 100 || point.z > 100){
                            point.x = point.x  * 0.1;
                            point.y = point.y  * 0.1;
                            point.z = point.z  * 0.1;
							if(min_scale > 0.1)
								min_scale = 0.1;
						}
                        else if(point.x > 10 || point.y > 10 || point.z > 10){
                            point.x = point.x  * 1.0;
                            point.y = point.y  * 1.0;
                            point.z = point.z  * 1.0;
							if(min_scale > 1.0)
								min_scale = 1.0;
						}
                        j++;
					}
					scale_factor = min_scale;
					reader.close();
                    imf.close();
					return 1;
				}
				else {
					cout<<"Error during reading file."<<endl;
				
					return 0;
				}
				
			} catch(E57Exception& ex){
                cout << "Error during reading file: " << ex.what() << endl;
                return -1;
			}
        }
	
        inline int saveE57File(const std::string &filename, PtrXYZ &cloud, float &scale_factor, int index = 0){
		try {
	        /// Open new file for writing, get the initialized root node (a Structure).
	        /// Path name: "/"
	        
	        ImageFile imf(filename, "w");
	        StructureNode root = imf.root();
	
            /// Register extension with URI=www.example.com/DemoExtension and prefix=demo
            //~ imf.extensionsAdd("Your Company", "https://www.example.com/DemoExtension");
	
	        /// Set per-file properties.
	        /// Path names: "/formatName", "/majorVersion", "/minorVersion", "/coordinateMetadata"
	        root.set("formatName", StringNode(imf, "ASTM E57 3D Imaging Data File"));
	        root.set("guid", StringNode(imf, "3F2504E0-4F89-11D3-9A0C-0305E82C3300"));
	
	        /// Get ASTM version number supported by library, so can write it into file
	        int astmMajor;
	        int astmMinor;
	        ustring libraryId;
	        E57Utilities().getVersions(astmMajor, astmMinor, libraryId);
	        root.set("versionMajor", IntegerNode(imf, astmMajor));
	        root.set("versionMinor", IntegerNode(imf, astmMinor));
	
	        /// Save a dummy string for coordinate system.
	        /// Really should be a valid WKT string identifying the coordinate reference system (CRS).
	        root.set("coordinateMetadata", StringNode(imf, "Cartesian Coordinate System"));
	
	        /// Create creationDateTime structure
	        /// Path name: "/creationDateTime
	        StructureNode creationDateTime = StructureNode(imf);
	        root.set("creationDateTime", creationDateTime);
	        time_t current_time;		//gets the EPOCH time (seconds elapsed since 1/1/1970)
			
	        creationDateTime.set("dateTimeValue", FloatNode(imf, time ( &current_time ))); //!!! convert time() to GPStime
	        
	        /// Create 3D data area.
	        /// Path name: "/data3D"
	        VectorNode data3D = VectorNode(imf, true);
	        root.set("data3D", data3D);
	
	        /// Add first scan
	        /// Path name: "/data3D/0"
	        StructureNode scan0 = StructureNode(imf);
	        data3D.append(scan0);
	
	        /// Add guid to scan0.
	        /// Path name: "/data3D/0/guid".
	        const char* scanGuid0 = "3F2504E0-4F89-11D3-9A0C-0305E82C3301";
	        scan0.set("guid", StringNode(imf, scanGuid0));
	
	        /// Make a prototype of datatypes that will be stored in points record.
	        /// This prototype will be used in creating the points CompressedVector.
	        /// Using this proto in a CompressedVector will define path names like:
	        ///      "/data3D/0/points/0/cartesianX"
            StructMaxMin vector[3];
	        findMaxMin(cloud, vector);
	        StructureNode proto = StructureNode(imf);
	        proto.set("cartesianX",  ScaledIntegerNode(imf, (double)vector[0].min, (double)vector[0].min, (double)vector[0].max, (double)scale_factor, 0));
	        proto.set("cartesianY",  ScaledIntegerNode(imf, (double)vector[1].min, (double)vector[1].min, (double)vector[1].max, (double)scale_factor, 0));
	        proto.set("cartesianZ",  ScaledIntegerNode(imf, (double)vector[2].min, (double)vector[2].min, (double)vector[2].max, (double)scale_factor, 0));
	        proto.set("cartesianInvalidState", IntegerNode(imf, 0, 0, 0));
	        proto.set("rowIndex",    IntegerNode(imf, 0, 0, 1));
	        proto.set("columnIndex", IntegerNode(imf, 0, 0, 1));
	        proto.set("returnIndex", IntegerNode(imf, 0, 0, 0));
            proto.set("returnCount", IntegerNode(imf, 1, 1, 1));

	        /// Make empty codecs vector for use in creating points CompressedVector.
	        /// If this vector is empty, it is assumed that all fields will use the BitPack codec.
	        VectorNode codecs = VectorNode(imf, true);
	
	        /// Create CompressedVector for storing points.  Path Name: "/data3D/0/points".
	        /// We use the prototype and empty codecs tree from above.
	        /// The CompressedVector will be filled by code below.
	        CompressedVectorNode points = CompressedVectorNode(imf, proto, codecs);
	        scan0.set("points", points);
	
	        /// Create pose structure for scan.
	        /// Path names: "/data3D/0/pose/rotation/w", etc...
	        ///             "/data3D/0/pose/translation/x", etc...
	        StructureNode pose = StructureNode(imf);
	        scan0.set("pose", pose);
	        StructureNode rotation = StructureNode(imf);
	        pose.set("rotation", rotation);
	        rotation.set("w", FloatNode(imf, 1.0));
	        rotation.set("x", FloatNode(imf, 0.0));
	        rotation.set("y", FloatNode(imf, 0.0));
	        rotation.set("z", FloatNode(imf, 0.0));
	        StructureNode translation = StructureNode(imf);
	        pose.set("translation", translation);
	        translation.set("x", FloatNode(imf, 0.0));
	        translation.set("y", FloatNode(imf, 0.0));
	        translation.set("z", FloatNode(imf, 0.0));
	
	      
	        /// Add name and description to scan
	        /// Path names: "/data3D/0/name", "/data3D/0/description".
	        scan0.set("name", StringNode(imf, "E57 Exporter by Michele Adduci"));
	        scan0.set("description", StringNode(imf, "Result"));
	
	        /// Add Cartesian bounding box to scan.
	        /// Path names: "/data3D/0/cartesianBounds/xMinimum", etc...
	        StructureNode bbox = StructureNode(imf);
	        bbox.set("xMinimum", FloatNode(imf, vector[0].min));
	        bbox.set("xMaximum", FloatNode(imf, vector[0].max));
	        bbox.set("yMinimum", FloatNode(imf, vector[1].min));
	        bbox.set("yMaximum", FloatNode(imf, vector[1].max));
	        bbox.set("zMinimum", FloatNode(imf, vector[2].min));
	        bbox.set("zMaximum", FloatNode(imf, vector[2].max));
	        scan0.set("cartesianBounds", bbox);
	
	        /// Add various sensor and version strings to scan.
	        /// Path names: "/data3D/0/sensorVendor", etc...
	        scan0.set("sensorVendor",           StringNode(imf, "Unknown"));
	        scan0.set("sensorModel",            StringNode(imf, "Unknown"));
	        scan0.set("sensorSerialNumber",     StringNode(imf, "MIC-ADDUCI"));
	        scan0.set("sensorHardwareVersion",  StringNode(imf, "1.0"));
	        scan0.set("sensorSoftwareVersion",  StringNode(imf, "1.0"));
	        scan0.set("sensorFirmwareVersion",  StringNode(imf, "1.0"));
	
	    
	        ///================
	        /// Prepare vector of source buffers for writing in the CompressedVector of points
	        int N = (int) cloud.get()->size();
	        cout<<"Number of point to write: "<<N<<endl;
	        double *cartesianX = new double[N];
	        double *cartesianY = new double[N];
	        double *cartesianZ = new double[N];
	        int32_t *cartesianInvalidState = new int32_t[N];
	        int32_t *rowIndex = new int32_t[N];
	        int32_t *columnIndex = new int32_t[N];
	        int32_t *returnIndex = new int32_t[N];
            int32_t *returnCount = new int32_t[N];
            for (auto j = 0; j < N; j++)
            {
				cartesianX[j] = cloud->points[j].x;	
				cartesianY[j] = cloud->points[j].y;	
                cartesianZ[j] = cloud->points[j].z;
				cartesianInvalidState[j] = 0;
				rowIndex[j] = 0;
				columnIndex[j] = 0;
				returnIndex[j] = 0;
				returnCount[j] = 1;
			}
	        std::vector<SourceDestBuffer> sourceBuffers;
	        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianX",  cartesianX,  N, true, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianY",  cartesianY,  N, true, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianZ",  cartesianZ,  N, true, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "cartesianInvalidState", cartesianInvalidState, N, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "rowIndex",    rowIndex,    N, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "columnIndex", columnIndex, N, true));
	        sourceBuffers.push_back(SourceDestBuffer(imf, "returnIndex", returnIndex, N, true));
            sourceBuffers.push_back(SourceDestBuffer(imf, "returnCount", returnCount, N, true));
			cout << "Source Buffers prepared"<< endl;
	        /// Write source buffers into CompressedVector
	        {
	            CompressedVectorWriter writer = points.writer(sourceBuffers);
	            writer.write(N);
	            writer.close();
				delete[] cartesianX;
	            delete[] cartesianY;
	            delete[] cartesianZ;
	            delete[] cartesianInvalidState;
	            delete[] rowIndex;
	            delete[] columnIndex;
	            delete[] returnIndex;
                delete[] returnCount;
	        }
	
	        imf.close();
	    } catch(E57Exception& ex) {
	        ex.report(__FILE__, __LINE__, __FUNCTION__);
	        return 0;
	    } catch (std::exception& ex) {
	        cout << "Got an std::exception, what=" << ex.what() << endl;
	        return 0;
	    } 
		
		return 1;
		}	
};

		
#endif
