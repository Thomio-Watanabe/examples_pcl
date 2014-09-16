#include<pcl/io/pcd_io.h>
#include<pcl/filters/radius_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


#include<iostream>


int main(int argc, char ** argv)
{
	// Object for storing the point clouds 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1],*cloud) != 0)
		return -1;
	
	// Filter object
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	
	// Every point must have 10 neighbors within 15cm, or it will be removed
	filter.setRadiusSearch(0.15);
	filter.setMinNeighborsInRadius(10);
	
	filter.filter(*filteredCloud);
	
	
	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Outlier removal"));
	viewer->addPointCloud<pcl::PointXYZ>(filteredCloud, "filteredCloud");
	// Display one normal out of 20, as a line of length 3cm.
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	
	
	return 1;
}
