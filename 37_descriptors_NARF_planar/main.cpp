// The following code will take a point cloud and create a range image from it, using spherical projection: 
// planar projection will give better results with clouds taken from depth camera:

#include<pcl/io/pcd_io.h>
#include<pcl/range_image/range_image_planar.h>
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

	// Parameters needed by the planar rang eimage object
	// Image size. Both kinect and xtion work at 640x480
	int imageSizeX = 640;
	int imageSizeY = 480;
	
	// Center of projection. Here, we choose the middle of the image 
	float centerX = 640.0f / 2.0f;
	float centerY = 480.0f / 2.0f;
	
	// Focal length. The value seen here has been taken from the original depth images. 
	// It is safe to use the same value vertically and horizontally. 
	float focalLengthX = 525.0f,focalLengthY = focalLengthX;
		
	// Sensor pose. Thankfully, the cloud includes the data
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
			cloud->sensor_origin_[1], cloud->sensor_origin_[2])) * Eigen::Affine3f(cloud->sensor_orientation_);
			
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius(i.e., 0.03 == 3 cm)
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored
	float minimumRange = 0.0f; 
	
	// Planar range image object
	pcl::RangeImagePlanar rangeImagePlanar;
	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud,imageSizeX,imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX, 
			sensorPose,pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);
	
	// Visualize the image
	pcl::visualization::RangeImageVisualizer viewer(" Range Image");
	viewer.showRangeImage(rangeImagePlanar);
	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU
		pcl_sleep(0.1);
	}

		
	return 0;
}
