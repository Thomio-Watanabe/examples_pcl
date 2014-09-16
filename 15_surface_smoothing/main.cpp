#include<pcl/io/pcd_io.h>
#include<pcl/surface/mls.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	// Object for storing the point clouds 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1],*cloud) != 0)
		return -1;
	
	// smoothing object object
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	
	// Use all neighbors in a radius of 3cm
	filter.setSearchRadius(0.03);
	filter.setPolynomialFit(true);
	filter.setComputeNormals(true);

	// kd-tree object for performing searches
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);

	
	filter.process(*smoothedCloud);
	
	
	// Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Smoothed Cloud"));
	viewer->addPointCloud<pcl::PointXYZ>(smoothedCloud, "smoothedCloud");
	// Display one normal out of 20, as a line of length 3cm.
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	
	
	return 1;
}
