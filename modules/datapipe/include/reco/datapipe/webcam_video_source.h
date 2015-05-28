/*
 * FrameProcessor.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_CVVIDEOSOURCE_H_
#define RECO_DATAPIPE_CVVIDEOSOURCE_H_

#pragma once

//local
#include "video_source.h"
#include "cv2_to_cv3_video.h"

//OpenCV
#include <opencv2/core/core.hpp>

namespace reco {
namespace datapipe {

/**
 * Retrieves and processes frames using the OpenCV VideoCapture interface.
 */
class webcam_video_source: public video_source {
public:
	webcam_video_source(unsigned int requestedWidth = 0, unsigned int requestedHeight = 0, int deviceNum = 0);
	virtual ~webcam_video_source();
	//fields
	unsigned int requestedWidth;
	unsigned int requestedHeight;
	//methods
public:
	virtual unsigned int get_width() {
		return requestedWidth;
	}
	virtual unsigned int get_height() {
		return requestedHeight;
	}
	//TODO: how can we effectively envelope this to protect it from misuse, i.e. calling retrieveFrame before setUp()?
	virtual bool set_up();
	virtual void tear_down();
	virtual bool capture_frame();
	bool setResolution(unsigned int width, unsigned int height);
	private:
	cv::VideoCapture camera;

	int deviceNum;
};
} /* namespace reco */
} /* namespace datapipe */

#endif /* RECO_DATAPIPE_CVVIDEOSOURCE_H_ */
