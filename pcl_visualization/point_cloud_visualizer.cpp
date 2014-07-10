#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// This function displays the help
void showHelp(char *program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h: Show this help." << std::endl; 
}

// Main function
int main(int argc,char **argv)
{
	// Show help
	if(pcl::console::find_switch(argc,argv,"-h") || pcl::console::find_switch(argc,argv,"--help"))
	{
		showHelp(argv[0]);
		return 0; 
	}
	
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;
	
	filenames = pcl::console::parse_file_extension_argument(argc,argv,".ply");
	
	if(filenames.size() != 1)
	{
		filenames = pcl::console::parse_file_extension_argument(argc,argv,".pcd");
		
		if(filenames.size() != 1)
		{
			showHelp(argv[0]);
			return -1;
		} else
		{
			file_is_pcd = true;
		}
	}
	
	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	
	if(file_is_pcd) {
		if(pcl::io::loadPCDFile(argv[filenames[0]],*source_cloud) < 0)
		{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		} 
	} else {
		if(pcl::io::loadPLYFile(argv[filenames[0]],*source_cloud) < 0)
		{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp (argv[0]);
			return -1;
		}
	}
	
	// Visualization 
	printf("\n Point cloud colors :\n"
		" \t white \t = \t original point cloud \n");
		
	pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");
	
	// Define R,G,B colors for the point cloud 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud,255,255,255); // White
	// We add the point cloud to the viewer and pass the color handler 
	viewer.addPointCloud(source_cloud,source_cloud_color_handler,"original_cloud");
	
	viewer.setBackgroundColor(0.05,0.05,0.05,0); // Set background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"original_cloud");
	
	while(!viewer.wasStopped()){   // Display the visualizer until the 'q' key is pressed
		viewer.spinOnce();
	}
	
	return 0;
} // End main()





















