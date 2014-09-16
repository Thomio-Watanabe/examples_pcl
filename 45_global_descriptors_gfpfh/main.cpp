/* The NARF descriptor encodes information about surface changes around a point. */

#include<pcl/io/pcd_io.h>
#include<pcl/features/gfpfh.h>


// Filter to downsamling 
#include<pcl/filters/voxel_grid.h>

// Filter to remove NANs
#include<pcl/filters/filter.h>

// Visualize the descriptors 
#include<pcl/visualization/histogram_visualizer.h> 
#include<pcl/visualization/cloud_viewer.h> 


int
main(int argc, char** argv)
{
	// Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZL>::Ptr object(new pcl::PointCloud<pcl::PointXYZL>);
	// Object for storing the GFPFH descriptor.
	pcl::PointCloud<pcl::GFPFHSignature16>::Ptr descriptor(new pcl::PointCloud<pcl::GFPFHSignature16>);
 
	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.
 
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZL>(argv[1], *object) != 0)
	{
		return -1;
	}
 
	// Note: you should now perform classification on the cloud's points. See the
	// original paper for more details. For this example, we will now consider 4
	// different classes, and randomly label each point as one of them.
	for (size_t i = 0; i < object->points.size(); ++i)
	{
		object->points[i].label = 1 + i % 4;
	}
 
	// ESF estimation object.
	pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
	gfpfh.setInputCloud(object);
	// Set the object that contains the labels for each point. Thanks to the
	// PointXYZL type, we can use the same object we store the cloud in.
	gfpfh.setInputLabels(object);
	// Set the size of the octree leaves to 1cm (cubic).
	gfpfh.setOctreeLeafSize(0.01);
	// Set the number of classes the cloud has been labelled with (default is 16).
	gfpfh.setNumberOfClasses(4);
 
	// Compute the features
	gfpfh.compute(*descriptor);
	
		
	return 0;
}
