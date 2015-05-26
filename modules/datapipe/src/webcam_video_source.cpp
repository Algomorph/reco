/*
 * FrameProcessor.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <reco/datapipe/webcam_video_source.h>
#include <exception>


namespace reco {
namespace datapipe {
WebcamVideoSource::WebcamVideoSource(unsigned int requestedWidth, unsigned int requestedHeight, int deviceNum):
		video_source(),
		requestedWidth(requestedWidth),
		requestedHeight(requestedHeight),
		camera(),
		deviceNum(deviceNum){
}

WebcamVideoSource::~WebcamVideoSource() {
	camera.release();
}

bool WebcamVideoSource::set_up(){
	return this->camera.open(this->deviceNum) && this->setResolution(this->requestedWidth,this->requestedHeight);
}

void WebcamVideoSource::tear_down(){
	this->camera.release();
}

bool WebcamVideoSource::capture_frame(){
	return this->camera.read(this->frame);
}
bool WebcamVideoSource::setResolution(unsigned int width, unsigned int height){
	//TODO: OpenCV3 support
	cv::Mat frame;
	camera >> frame;
	bool horizChange = camera.set(CV_CAP_PROP_FRAME_WIDTH, (double)width);
	bool vertChange = camera.set(CV_CAP_PROP_FRAME_HEIGHT, (double)height);
	return horizChange && vertChange;
}

} /* namespace datapipe */
} /* namespace reco */
