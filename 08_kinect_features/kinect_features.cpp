#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>


 class SimpleOpenNIViewer
 {
   private:
   		pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> ne;	// Create the normal estimation class
   		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
   		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree;

   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer")
     { 
     	cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
     	tree = pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
     }

     void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
		// pass the input dataset to a normal estimation object
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.003);

		// Compute the features
		
		ne.compute(*cloud_normals);
	
		if (!viewer.wasStopped())
			viewer.showCloud (cloud);
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
 
 
 



