#pragma once

const int POINTS_WIDTH = 512;
const int POINTS_HEIGHT = 424;
const int COLOR_WIDTH = 1920;
const int COLOR_HEIGHT = 1080;

class Color {
public:
	Color() : r(0), g(0), b(0) {};
	Color(float r, float g, float b) : r(r), g(g), b(b) { }
	float r, g, b;
};

class MyKinect
{
public:
	MyKinect();
	~MyKinect();

private:
	bool kinect_init();
	void get_depth_data(IMultiSourceFrame* frame);
	void get_rgb_data(IMultiSourceFrame* frame);

	std::vector<CameraSpacePoint> depth_points;
	std::vector<ColorSpacePoint> color_points;
	std::vector<uint8_t> image_data;
	std::vector<Color> image_colors;

	IKinectSensor * sensor;             // Kinect sensor
	IMultiSourceFrameReader* reader;   // Kinect data source
	ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates
};

