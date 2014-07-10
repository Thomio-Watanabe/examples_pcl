#ifndef FEATURES_HPP_
#define FEATURES_HPP_

#include <pcl/io/pcd_io.h>		// pcl::io::loadPCDFile
#include <pcl/io/ply_io.h>		// pcl::io::loadPLYFile
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h> // PCLVisualizer class
#include <iostream>
#include <cstdlib>  	//exit, EXIT_FAILURE

class InputCloud{
	private:
		int showHelp(char* program_name);
		bool file_is_pcd;
		std::vector<int> filenames;
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;	// Create the normal estimation class
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;		// Output dataset
	public:
		InputCloud();
		~InputCloud();
		int checkInput(int,char**);
		int loadDataset(char**);
		int calc3dFeatures(void);
		int visualizeCloud(void);
};

#endif
