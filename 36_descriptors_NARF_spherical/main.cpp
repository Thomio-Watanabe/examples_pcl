// The following code will take a point cloud and create a range image from it, using spherical projection: 
// e spherical projection give us an image similar to the ones produced by a LIDAR sensor.

#include<pcl/io/pcd_io.h>
#include<pcl/range_image/range_image.h>
#include<pcl/visualization/range_image_visualizer.h>


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


	// Angular resolution is the angular distance between pixels.
	// Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
	// Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
	float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
	float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
	
	// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
	// the first would be 360º. Choosing values that adjust to the real sensor will
	// decrease the time it takes, but don't worry. If the values are bigger than
	// the real ones, the image will be automatically cropped to discard empty zones.
	float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
	float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
	
	// Sensor pose. Thankfully, the cloud includes the data
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
			cloud->sensor_origin_[1], cloud->sensor_origin_[2])) * Eigen::Affine3f(cloud->sensor_orientation_);
			
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius(i.e., 0.03 == 3 cm)
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored
	float minimumRange = 0.0f; 
	// Border size. If grater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;
	
	// Range image object
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud,angularResolutionX,angularResolutionY,
			maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange, borderSize);
	
	// Visualize the point cloud
	pcl::visualization::RangeImageVisualizer viewer(" Range Image");
	viewer.showRangeImage(rangeImage);
	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU
		pcl_sleep(0.1);
	}

		
	return 0;
}
