/* The NARF descriptor encodes information about surface changes around a point. */

#include<pcl/io/pcd_io.h>
#include<pcl/range_image/range_image_planar.h>
#include<pcl/features/range_image_border_extractor.h>
#include<pcl/keypoints/narf_keypoint.h>
#include<pcl/features/narf_descriptor.h>


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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the keypoints indices 
	pcl::PointCloud<int>::Ptr keypoints(new pcl::PointCloud<int>);
	// Object for storing the NARF descriptors
	pcl::PointCloud<pcl::Narf36>::Ptr descriptors(new pcl::PointCloud<pcl::Narf36>);
	
		
	// Read a PCD file from disk
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) != 0)
	{
		return -1;
	}

/*
	// Removing NANs from the input cloud
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloudColor,*cloudColor,mapping);

	// Downsampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Filter object
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(0.01f,0.01f,0.01f);
	filter.filter(*filteredCloud);
*/

	// Convert the cloud to range image
	int imageSizeX = 640;
	int imageSizeY = 480;
	float centerX = 640.0f / 2.0f;
	float centerY = 480.0f / 2.0f;
	float focalLengthX = 525.0f,focalLengthY = focalLengthX;
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
			cloud->sensor_origin_[1], cloud->sensor_origin_[2])) * Eigen::Affine3f(cloud->sensor_orientation_);
	float noiseLevel = 0.0f;
	float minimumRange = 0.0f; 
	
	// Planar range image object
	pcl::RangeImagePlanar rangeImage;
	rangeImage.createFromPointCloudWithFixedSize(*cloud,imageSizeX,imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX, 
			sensorPose,pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);
			
	// Extract the keypoints
	pcl::RangeImageBorderExtractor borderExtractor;	
	// Keypoints detection object
	pcl::NarfKeypoint detector(&borderExtractor);
	detector.setRangeImage(&rangeImage);	
	// The support size influences how big the surface of interest will be, 
	// when finding keypoints from the border information
	detector.getParameters().support_size = 0.2f;	
	detector.compute(*keypoints);
		
		
	
	// The NARF estimator needs to indices in a vector, not a cloud 
	std::vector<int> keypoints2;
	keypoints2.resize(keypoints->points.size());
	for(unsigned int i=0; i < keypoints->size(); ++i)
		keypoints2[i] = keypoints->points[i];
	// Narf estimation object
	pcl::NarfDescriptor narf(&rangeImage,&keypoints2);
	// Support size: choose the same value you used for keypoint extraction
	narf.getParameters().support_size = 0.2f;	
	// If true, the rotation invariant version of NARF will be used. The histogram
	// will be shifted according to the dominant orientation to provide robustness to
	// rotations around the normal.
	narf.getParameters().rotation_invariant = true;
	
	narf.compute(*descriptors);
		
	return 0;
}
