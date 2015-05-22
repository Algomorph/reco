/*
 * FrameProcessor.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <reco/vidgui/WebcamVideoSource.h>

//std
#include <exception>


namespace reco {
namespace vidgui {
WebcamVideoSource::WebcamVideoSource(unsigned int requestedWidth, unsigned int requestedHeight, int deviceNum):
		VideoSource(),
		requestedWidth(requestedWidth),
		requestedHeight(requestedHeight),
		camera(),
		deviceNum(deviceNum){
}

WebcamVideoSource::~WebcamVideoSource() {
	camera.release();
}

bool WebcamVideoSource::setUp(){
	return this->camera.open(this->deviceNum) && this->setResolution(this->requestedWidth,this->requestedHeight);
}

void WebcamVideoSource::tearDown(){
	this->camera.release();
}

cv::Mat WebcamVideoSource::retrieveFrame(){
	cv::Mat frame;
	this->camera.read(frame);
	return frame;
}
bool WebcamVideoSource::setResolution(unsigned int width, unsigned int height){
	//TODO: OpenCV3 support
	cv::Mat frame;
	camera >> frame;
	//camera.set(CV_CAP_PROP_FOURCC,CV_FOURCC('H','2','6','4'));
	bool horizChange = camera.set(CV_CAP_PROP_FRAME_WIDTH, (double)width);
	bool vertChange = camera.set(CV_CAP_PROP_FRAME_HEIGHT, (double)height);
	return horizChange && vertChange;
}

} /* namespace vidgui */
} /* namespace reco */
