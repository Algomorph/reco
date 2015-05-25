/*
 * FrameProcessor.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_CVVIDEOSOURCE_H_
#define MODULES_VIDEO_CVVIDEOSOURCE_H_

#pragma once

//local
#include "VideoSource.h"
#include "Cv2To3Video.h"

//OpenCV
#include <opencv2/core/core.hpp>

namespace reco {
namespace datapipe {

/**
 * Retrieves and processes frames using the OpenCV VideoCapture interface.
 */
class WebcamVideoSource: public VideoSource {
public:
	WebcamVideoSource(unsigned int requestedWidth = 0, unsigned int requestedHeight = 0, int deviceNum = 0);
	virtual ~WebcamVideoSource();
	//fields
	unsigned int requestedWidth;
	unsigned int requestedHeight;
	//methods
public:
	virtual unsigned int getWidth() {
		return requestedWidth;
	}
	virtual unsigned int getHeight() {
		return requestedHeight;
	}
	//TODO: how can we effectively envelope this to protect it from misuse, i.e. calling retrieveFrame before setUp()?
	virtual bool setUp();
	virtual void tearDown();
	virtual cv::Mat retrieveFrame();
	bool setResolution(unsigned int width, unsigned int height);
	private:
	cv::VideoCapture camera;

	int deviceNum;
};
} /* namespace video */
} /* namespace augmentarium */

#endif /* MODULES_VIDEO_CVVIDEOSOURCE_H_ */
