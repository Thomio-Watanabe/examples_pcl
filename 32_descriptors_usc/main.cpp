
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/usc.h>

// Filter to downsamling 
#include<pcl/filters/voxel_grid.h>

// Filter to remove NANs
#include<pcl/filters/filter.h>

// Visualize the descriptors 
#include<pcl/visualization/cloud_viewer.h> 


int main(int argc, char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the USC descriptors for each point
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}

	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,mapping);

	// Downsampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Filter object
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f,0.01f,0.01f);
	filter.filter(*filteredCloud);


		
	// USC estimation object
	pcl::UniqueShapeContext<pcl::PointXYZ,pcl::ShapeContext1980,pcl::ReferenceFrame> usc;
	usc.setInputCloud(filteredCloud);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere
	usc.setRadiusSearch(0.05);
	// Set the minimal radius value for the search sphere, to avoid being too sensitive 
	// in bins close to the center of the sphere
	usc.setMinimalRadius(0.05/10.0);
	
	// Radius used to compute the local point densityfor the nighbors 
	// (the density is the number of points within that radius).
	usc.setPointDensityRadius(0.05 / 5.0);
	
	// Set the radius to compute the Local Reference Frame
	usc.setLocalRadius(0.05);
	
	usc.compute(*descriptors);
	
	/*
	// Visualize the descriptors ???
	pcl::visualization::CloudViewer viewerDescriptors(" Visualize Descriptors");
	viewerDescriptors.showCloud(descriptors);
	while(!viewerDescriptors.wasStopped())
	{
		// Do nothing
	}
	*/		
	
	return 0;
}
