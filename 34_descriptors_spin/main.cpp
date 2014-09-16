
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/spin_image.h>

// Filter to downsamling 
#include<pcl/filters/voxel_grid.h>

// Filter to remove NANs
#include<pcl/filters/filter.h>

// Visualize the descriptors 
#include<pcl/visualization/histogram_visualizer.h> 
#include<pcl/visualization/cloud_viewer.h> 

// A handy typedef
typedef pcl::Histogram<153> SpinImage; 


int main(int argc, char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object to store the normals 
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point
	pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}


	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,mapping);
/*
	// Downsampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Filter object
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f,0.01f,0.01f);
	filter.filter(*filteredCloud);
*/

	// Estimate normals 
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
		
	// spin image estimation object
	pcl::SpinImageEstimation<pcl::PointXYZ,pcl::Normal,SpinImage> si;
	si.setInputCloud(cloud);
	si.setInputNormals(normals);
	// Radius of the suport cylinder
	si.setRadiusSearch(0.02);
	// Set the resolution of the spin image (the number of bins along one dimension).
	// Note: you must change the output histogram size to reflect this
	si.setImageWidth(8);
	
	si.compute(*descriptors);
	
	
	// Visualize the descriptors
	pcl::visualization::PCLHistogramVisualizer viewerDescriptors;
	viewerDescriptors.addFeatureHistogram(*descriptors,153);
	viewerDescriptors.spin();

	// Visualize the descriptors ???
	pcl::visualization::CloudViewer viewer(" Visualize Descriptors");
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{
		// Do nothing
	}
		
	return 0;
}
