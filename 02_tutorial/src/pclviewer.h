#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream> 
#include <QMainWindow> 
//Point Cloud Libraries 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h> 
// Visualization toolkit (VTK)
#include <vtkRenderer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
	class PCLViewer;
}

class PCLViewer : public QMainWindow
{ 
	Q_OBJECT
	public:
		explicit PCLViewer (QWidget *parent = 0);
		~PCLViewer();
	public slots:
		void RGBsliderReleased();
		void randomButtonPressed();
		void pSliderValueChanged(int value);
		void redSliderValueChanged(int value);
		void greenSliderValueChanged(int value);
		void blueSliderValueChanged(int value);
	protected:
		boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer;
		PointCloudT::Ptr cloud;
		unsigned int red;
		unsigned int green;
		unsigned int blue;
	private:
    	Ui::PCLViewer *ui;
};
#endif

