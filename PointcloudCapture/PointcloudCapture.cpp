// PointcloudCapture.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <conio.h>
#include <ctype.h>

#include <pcl/visualization/cloud_viewer.h>

#include "MyKinect.h"


int main()
{
	MyKinect kinect;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(kinect.get_pointcloud()));

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {
		*cloud = kinect.get_pointcloud();
	}

	std::cout << "Press any key to continue..." << std::flush;
	_getch();
    return 0;
}

