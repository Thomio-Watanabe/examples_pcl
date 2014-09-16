#include<pcl/io/pcd_io.h>
#include<pcl/octree/octree.h>

#include<iostream>

int main(int argc, char ** argv)
{
	// Object for storing the point clouds 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

	// Read two PCD files from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloudA) != 0)
		return -1;
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2],*cloudB) != 0)
		return -1;
	
	// Create octree object with lowest resolution
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(128.0f);
	
	// Add cloudA to the octree
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	// add cloudB to octree
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();
	
	std::vector<int> newPoints; 
	octree.getPointIndicesFromNewVoxels(newPoints);
	
	for(size_t i = 0; i < newPoints.size(); ++i)
	{
		std::cout << "Point (" << cloudB->points[newPoints[i]].x << ", "
					<< cloudB->points[newPoints[i]].y << ", "
					<< cloudB->points[newPoints[i]].z
					<<") was not in cloud A but is in cloud B" << std::endl;		
	}

	std::cout << newPoints.size() <<std::endl;
		
	return 1;
}
