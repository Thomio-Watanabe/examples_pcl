// the region growing use normals to estimate clusters 

#include<pcl/io/pcd_io.h>
#include<pcl/search/kdtree.h>
#include<pcl/segmentation/region_growing_rgb.h>

#include<iostream> 

int main(int argc,char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1],*cloud) != 0)
	{
		return -1;
	}


	// kd-tree objects for searches
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	kdtree->setInputCloud(cloud);
	
	// Color-based region growing clustering object
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud);
	clustering.setSearchMethod(kdtree);
	clustering.setMinClusterSize(20);
	clustering.setDistanceThreshold(5);
	clustering.setPointColorThreshold(6);
	clustering.setRegionColorThreshold(5);

	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);
	
	// For every cluster 
	int currentClusterNum = 1;
	for(std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// add all its points to a new cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(std::vector<int>::const_iterator point = i->indices.begin();point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
		
		// save it to disk
		if(cluster->points.size() <= 0)
			break;
		std::cout << " Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName,*cluster);
		
		currentClusterNum++;
	}	
		
	return 0; 
}
