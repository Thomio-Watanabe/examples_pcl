
#include<pcl/io/pcd_io.h>
#include<pcl/point_types_conversion.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/intensity_gradient.h>
#include<pcl/features/rift.h>

// Filter to downsamling 
#include<pcl/filters/voxel_grid.h>

// Filter to remove NANs
#include<pcl/filters/filter.h>

// Visualize the descriptors 
#include<pcl/visualization/histogram_visualizer.h> 
#include<pcl/visualization/cloud_viewer.h> 

// A handy typedef
typedef pcl::Histogram<32> RIFT32; 


int main(int argc, char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Object to store the point cloud with intensity value
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity(new pcl::PointCloud<pcl::PointXYZI>);
	// Object to store intensity gradients 
	pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
	// Object to store the normals 
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the RIFT descriptors for each point
	pcl::PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloudColor) != 0)
	{
		return -1;
	}


	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloudColor,*cloudColor,mapping);
/*
	// Downsampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Filter object
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f,0.01f,0.01f);
	filter.filter(*filteredCloud);
*/

	// Convert the RGB to intensity
	pcl::PointCloudXYZRGBtoXYZI(*cloudColor,*cloudIntensity);


	// Estimate normals 
	pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloudIntensity);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	
	// Compute the intensity gradients 
	pcl::IntensityGradientEstimation<pcl::PointXYZI,pcl::Normal,pcl::IntensityGradient,
			pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
	ge.setInputCloud(cloudIntensity);
	ge.setInputNormals(normals);
	ge.setRadiusSearch(0.03);
	ge.compute(*gradients);
		
	
	// RIFT estimation object
	pcl::RIFTEstimation<pcl::PointXYZI,pcl::IntensityGradient,RIFT32> rift;
	rift.setInputCloud(cloudIntensity);
	rift.setSearchMethod(kdtree);
	// Set intensity gradients to use 
	rift.setInputGradient(gradients);
	// Radius to get all neigbors within
	rift.setRadiusSearch(0.02);
	// Set the number of bins to use in the distance dimension
	rift.setNrDistanceBins(4);
	// Set the number of bins to use in the gradient orientation dimension
	rift.setNrGradientBins(8);
	// Note: you must change the outputhistogram size to reflext the previous values.
	
	rift.compute(*descriptors);
	
	
	// Visualize the descriptors
	pcl::visualization::PCLHistogramVisualizer viewerDescriptors;
	viewerDescriptors.addFeatureHistogram(*descriptors,32);
	viewerDescriptors.spin();
	
	/*
	// Visualize the point cloud
	pcl::visualization::CloudViewer viewer(" Visualize Descriptors");
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{
		// Do nothing
	}
	*/
		
	return 0;
}
