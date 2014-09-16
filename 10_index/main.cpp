#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
 
#include <vector>
 
int main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtracted(new pcl::PointCloud<pcl::PointXYZ>);
 
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudAll) != 0)
	{
		return -1;
	}
 
	// Plane segmentation (do not worry, we will see this later).
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setOptimizeCoefficients(true);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setInputCloud(cloudAll);
 
	// Object for storing the indices.
	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
 
	segmentation.segment(*pointIndices, *coefficients);
 
	// Object for extracting points from a list of indices.
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloudAll);
	extract.setIndices(pointIndices);
	// We will extract the points that are NOT indexed (the ones that are not in a plane).
	extract.setNegative(true);
	extract.filter(*cloudExtracted);
}
