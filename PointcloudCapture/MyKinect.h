#pragma once
class MyKinect
{
public:
	MyKinect();
	~MyKinect();

private:
	bool kinect_init();

	IKinectSensor * sensor;             // Kinect sensor
	IMultiSourceFrameReader* reader;   // Kinect data source
	ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates
};

