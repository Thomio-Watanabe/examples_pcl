// the region growing use normals to estimate clusters 

#include<pcl/io/pcd_io.h>
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/region_growing.h>

#include<iostream> 

int main(int argc,char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals 
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}


	// kd-tree objects for searches
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);
	
	// Estimate Normals
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	
	
	
	// Region growing clustering object
	pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> clustering;
	clustering.setMinClusterSize(50);
	clustering.setMaxClusterSize(10000);
	clustering.setSearchMethod(kdtree);
	clustering.setNumberOfNeighbours(20);
	clustering.setInputCloud(cloud);
	clustering.setInputNormals(normals);
	// Set the angle in radians that will be the smoothness threshold
	// (the maximum allowable deviation of the normals)
	clustering.setSmoothnessThreshold( (20/180) * 3.1415); // 7 degrees
	// Set the curvature threshold. The disparity between curvatures will be 
	// tested after the normal deviation check has passed. 
	clustering.setCurvatureThreshold(1.0);
	
	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);
	
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
