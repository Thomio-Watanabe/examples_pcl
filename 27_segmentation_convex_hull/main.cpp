// The following code shows how you can use PCL to compute the convex hull of the first plane found in the scene
// There is no need to set the alpha parameter

#include<pcl/io/pcd_io.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/convex_hull.h>
#include<pcl/visualization/cloud_viewer.h> 

int main(int argc,char** argv)
{
	// Object for storing the point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}

	// Object for storing the plane model coefficients 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	// Configure the object to look for a cylinder 
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	// Use RANSAC method 
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed dstance to the model
	segmentation.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional)
	segmentation.setOptimizeCoefficients(true);
	
	pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
	segmentation.segment(*inlierIndices,*coefficients);

	if(inlierIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl; 
	else
	{
		// Copy the point of the plane to a new cloud 
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inlierIndices);
		extract.filter(*plane);

		// Object for retrieving the concave hull
		pcl::ConvexHull<pcl::PointXYZ> hull;
		pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
		hull.setInputCloud(plane);
		
		hull.reconstruct(*convexHull);
				
		// Visualize the hull
		pcl::visualization::CloudViewer viewerPlane(" Concave hull");
		viewerPlane.showCloud(convexHull);
		while(!viewerPlane.wasStopped())
		{
			// Do nothing
		}
	}
	return 0; 
}
