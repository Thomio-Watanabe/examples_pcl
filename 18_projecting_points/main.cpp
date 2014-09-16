#include<pcl/io/pcd_io.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/project_inliers.h>
#include<pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	// Objects for storing the point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoPlane(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr planePoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
		return -1;

	// Get the plane model, if present 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
	segmentation.segment(*inlierIndices,*coefficients);

	if(inlierIndices->indices.size() == 0)
		std::cout << " Could not find a plane in the scene." <<std::endl;
	else
	{
		std::cerr << " Plane coefficients: " << coefficients->values[0] << " "
						<< coefficients->values[1] << " "
						<< coefficients->values[2] << " "
						<< coefficients->values[3] << std::endl; 
		
		// Create a second point cloud that does not have the plane points 
		// Also, extract the plane points to visualize them later 
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inlierIndices);
		extract.filter(*planePoints);
		extract.setNegative(true);
		extract.filter(*cloudNoPlane);
		
		// Object for projecting points onto a model
		pcl::ProjectInliers<pcl::PointXYZ> projection;
		// We have to specify what model we used
		projection.setModelType(pcl::SACMODEL_PLANE);
		projection.setInputCloud(cloudNoPlane);
		// And we have to give the coefficients we got
		projection.setModelCoefficients(coefficients);
		projection.filter(*projectedPoints);
		
		
		// Visualize everything
		pcl::visualization::CloudViewer viewerPlane("Plane");
		viewerPlane.showCloud(planePoints);
		while(!viewerPlane.wasStopped())
		{
			// Do nothing but wait
		}
				
		
		pcl::visualization::CloudViewer viewerProjection("Projected points");
		viewerProjection.showCloud(projectedPoints);
		while(!viewerProjection.wasStopped())
		{
			// Do nothing but wait
		}
	} // end else

	return 0;
}
