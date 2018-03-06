#include "stdafx.h"
#include "MyKinect.h"


MyKinect::MyKinect()
{
	if (!kinect_init()) {
		std::cerr << "Could not initialise kinect. gg" << std::endl;
	}
}

bool MyKinect::kinect_init() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
			&reader);
		return reader;
	}
	else {
		return false;
	}
}


MyKinect::~MyKinect()
{
}
