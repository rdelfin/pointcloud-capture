#include "stdafx.h"
#include "MyKinect.h"


MyKinect::MyKinect()
{
	if (!kinect_init()) {
		std::cerr << "Could not initialise kinect. gg" << std::endl;
	}
}

bool MyKinect::kinect_init() {
	if (FAILED(GetDefaultKinectSensor(&this->sensor))) {
		return false;
	}
	if (this->sensor) {
		this->sensor->get_CoordinateMapper(&this->mapper);

		this->sensor->Open();
		this->sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
			&this->reader);
		return this->reader != NULL;
	}
	else {
		return false;
	}
}

const pcl::PointCloud<pcl::PointXYZRGB>& MyKinect::get_pointcloud() {
	this->get_kinect_data();
	return this->cloud;
}

void MyKinect::get_kinect_data() {
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		this->get_depth_data(frame);
		this->get_rgb_data(frame);
		this->cloud.clear();
		this->cloud.width = POINTS_WIDTH;
		this->cloud.height = POINTS_HEIGHT;
		this->cloud.is_dense = false;
		this->cloud.resize(POINTS_WIDTH*POINTS_HEIGHT);

		for (size_t i = 0; i < POINTS_WIDTH*POINTS_HEIGHT; i++) {
			pcl::PointXYZRGB point;
			point.x = depth_points[i].X;
			point.y = depth_points[i].Y;
			point.z = depth_points[i].Z;
			point.r = image_colors[i].r;
			point.g = image_colors[i].g;
			point.b = image_colors[i].b;
			this->cloud.points[i] = point;
		}
	}
	if (frame) frame->Release();
}

void MyKinect::get_depth_data(IMultiSourceFrame* frame) {
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) frameref->Release();

	if (!depthframe) return;

	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);
	
	const int IMG_SIZE = POINTS_WIDTH * POINTS_HEIGHT;
	this->depth_points.resize(IMG_SIZE);
	this->color_points.resize(IMG_SIZE);

	// Write vertex coordinates
	mapper->MapDepthFrameToCameraSpace(IMG_SIZE, buf, IMG_SIZE, &this->depth_points[0]);

	// Fill in depth2rgb map
	mapper->MapDepthFrameToColorSpace(IMG_SIZE, buf, IMG_SIZE, &this->color_points[0]);
	if (depthframe) depthframe->Release();
}


void MyKinect::get_rgb_data(IMultiSourceFrame* frame) {
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) frameref->Release();

	if (!colorframe) return;

	const int IMG_SIZE = COLOR_WIDTH * COLOR_HEIGHT;
	this->image_data.resize(IMG_SIZE * 4);
	this->image_colors.resize(POINTS_WIDTH*POINTS_HEIGHT);

	// Get data from frame
	colorframe->CopyConvertedFrameDataToArray(IMG_SIZE * 4, &this->image_data[0], ColorImageFormat_Rgba);
	
	// Write color array for vertices
	for (int i = 0; i < POINTS_WIDTH*POINTS_HEIGHT; i++) {
		ColorSpacePoint p = this->color_points[i];
		// Check if color pixel coordinates are in bounds
		// Intentionally skipping the alpha channel
		if (p.X < 0 || p.Y < 0 || p.X > COLOR_WIDTH || p.Y > COLOR_HEIGHT) {
			this->image_colors[i] = Color();
		}
		else {
			int idx = (int)p.X + COLOR_WIDTH * (int)p.Y;
			this->image_colors[i] = Color(
				this->image_data[4 * idx + 0] / 255.,
				this->image_data[4 * idx + 1] / 255.,
				this->image_data[4 * idx + 2] / 255.);
		}
	}

	if (colorframe) colorframe->Release();
}


MyKinect::~MyKinect()
{
}
