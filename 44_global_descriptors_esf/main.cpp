/* The NARF descriptor encodes information about surface changes around a point. */

#include<pcl/io/pcd_io.h>
#include<pcl/features/esf.h>


// Filter to downsamling 
#include<pcl/filters/voxel_grid.h>

// Filter to remove NANs
#include<pcl/filters/filter.h>

// Visualize the descriptors 
#include<pcl/visualization/histogram_visualizer.h> 
#include<pcl/visualization/cloud_viewer.h> 



int main(int argc, char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the OUR-CVFH descriptor
	pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
			
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*object) != 0)
	{
		return -1;
	}

/*

	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*object,*object,mapping);

	// Downsampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Filter object
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f,0.01f,0.01f);
	filter.filter(*filteredCloud);


	// Estimate normals
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(object);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
*/


	// ESF estimation object
	pcl::ESFEstimation<pcl::PointXYZ,pcl::ESFSignature640> esf;
	esf.setInputCloud(object);
	
	esf.compute(*descriptor);
		
	return 0;
}
