/* The NARF descriptor encodes information about surface changes around a point. */

#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/cvfh.h>


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
	// Object for storing the normals  
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the VFH descriptor
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
			
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*object) != 0)
	{
		return -1;
	}


	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*object,*object,mapping);

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
	normalEstimation.setInputCloud(object);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	
	// CVFH estimation object
	pcl::CVFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> cvfh;
	cvfh.setInputCloud(object);
	cvfh.setInputNormals(normals);
	cvfh.setSearchMethod(kdtree);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step. 
	cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI);   // 5 degrees
	// Set the curvature threshold ( maximum disparity between curvatures),
	// for the region segmentaion step
	cvfh.setCurvatureThreshold(1.0);
	
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite 
	cvfh.setNormalizeBins(false);
	
	
	cvfh.compute(*descriptors);
		
	return 0;
}
