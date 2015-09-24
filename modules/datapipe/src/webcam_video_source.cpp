/*
 * FrameProcessor.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//TODO: 100 get rid of old pipeline / video-source classes after next master merge

//local
#include <reco/datapipe/webcam_video_source.h>
#include <exception>


namespace reco {
namespace datapipe {
webcam_video_source::webcam_video_source(unsigned int requestedWidth, unsigned int requestedHeight, int deviceNum):
		video_source(),
		requestedWidth(requestedWidth),
		requestedHeight(requestedHeight),
		camera(),
		deviceNum(deviceNum){
}

webcam_video_source::~webcam_video_source() {
	camera.release();
}

bool webcam_video_source::set_up(){
	return this->camera.open(this->deviceNum) && this->setResolution(this->requestedWidth,this->requestedHeight);
}

void webcam_video_source::tear_down(){
	this->camera.release();
}

bool webcam_video_source::capture_frame(){
	return this->camera.read(this->frame);
}
bool webcam_video_source::setResolution(unsigned int width, unsigned int height){
	cv::Mat frame;
	camera >> frame;
	bool horizChange = camera.set(CV_CAP_PROP_FRAME_WIDTH, (double)width);
	bool vertChange = camera.set(CV_CAP_PROP_FRAME_HEIGHT, (double)height);
	return horizChange && vertChange;
}

} /* namespace datapipe */
} /* namespace reco */
