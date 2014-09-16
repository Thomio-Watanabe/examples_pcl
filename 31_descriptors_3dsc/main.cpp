
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/3dsc.h>

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
	// Object for storing the normals 
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the PFH descriptors for each point
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


		
	// Estimate the normals 
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(filteredCloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	
	// 3DSC estimation object
	pcl::ShapeContext3DEstimation<pcl::PointXYZ,pcl::Normal,pcl::ShapeContext1980> sc3d;
	sc3d.setInputCloud(filteredCloud);
	sc3d.setInputNormals(normals);
	sc3d.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere
	sc3d.setRadiusSearch(0.05);
	// Set the minimal radius value for the search sphere, to avoid being too sensitive 
	// in bins close to the center of the sphere
	sc3d.setMinimalRadius(0.05/2.0);
	
	sc3d.compute(*descriptors);
	
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
