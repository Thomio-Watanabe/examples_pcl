#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

int main(void)
{
	// Variables
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>()); 

	
	
	
	// load point cloud
	pcl::io::loadPCDFile("../datasets/table_scene_mug_stereo_textured.pcd",*cloud);
		
	
	
	
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;	// Create the normal estimation class	
	// pass the input dataset to a normal estimation object
	ne.setInputCloud(cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object
	// Its content will be filled inside the object, based on the given input dataset ( as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree1);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);
	// Compute the features
	ne.compute(*normals);
	
	
	
	
		
	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	// Alternatively, if cloud is of type PointNormal, do pfh.setInputNormals(cloud);
	
	
	
	
	// Create an empty kdtree representation, and pass it to the PFH estimation object
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//pcl::KdTreeFLANN<cl::PointXYZ>:: Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());	-- older call for PCL 1.5
	pfh.setSearchMethod(tree);
	
	// Output datasets 
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
	
	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch(0.05);


	for(int i = 0; i < normals->points.size() ; i++)
	{
		if(!pcl::isFinite<pcl::Normal>(normals->points[i]))	
		{
			PCL_WARN("normals[%d] is not finite \n",i);
		}
	}


	
	// Compute the features
	pfh.compute(*pfhs);
	
	// pfhs->points.size() should have the same size as the input cloud -> points.size()
	
	
	
	
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 05);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
	
	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	
	
	
	return 0;	

}
