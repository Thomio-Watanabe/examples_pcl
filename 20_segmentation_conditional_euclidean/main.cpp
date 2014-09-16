// The euclidean segmentation is the simplest among segmentations techniques
// It checks between two points if it is less than a threshold, both are considered to belong to the same cluster. 

#include<pcl/io/pcd_io.h>
#include<pcl/segmentation/conditional_euclidean_clustering.h>
#include<iostream> 

// if this subfunction returns true, the candidate point will be added 
// to the cluster of the seed point
bool customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// Do whatever you want here
	if(candidatePoint.y < seedPoint.y)
		return false;
	
	return true;
}



int main(int argc,char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}

	
	// Conditional Euclidean clustering object
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	// Set cluster tolerance to 2 cm (smaller values may cause objects to be divided
	//in several cluster, whereas big values may join objects in a single cluster 
	clustering.setClusterTolerance(0.02);
	// Set the minimum and maximum number of points that a cluster can have 
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);
	
	
	/*
	// kd-tree objects for searches
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);
	clustering.setSearchMethod(kdtree);
	clustering.extract(clusters);

	*/

	// For every cluster 
	int currentClusterNum = 1;
	for(std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// add all its points to a new cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
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
