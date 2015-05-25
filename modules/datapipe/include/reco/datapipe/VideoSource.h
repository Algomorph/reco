/*
 * FrameProcessor.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_FRAMEPROCESSOR_H_
#define MODULES_VIDEO_FRAMEPROCESSOR_H_

#pragma once

//OpenCV
#include <opencv2/core/core.hpp>
#include <QObject>
//local
#include "StaticTypeRegistration.h"
//standard
//#include <stdarg.h>

namespace reco {
namespace datapipe {

/**
 * Captures and processes each subsequent frame of the video
 */
class VideoSource: public QObject {
	Q_OBJECT
protected:
	VideoSource()
	{}
	virtual ~VideoSource(){}

public:
	virtual unsigned getWidth() = 0;
	virtual unsigned getHeight() = 0;

/**
 * Prepare camera for capture or video source for playback
 */
	virtual bool setUp() = 0;
/**
 * Stop the capture, release camera / deallocate video source
 */
	virtual void tearDown() = 0;
/**
 * Retrieve a single frame (assumes being called between setUpCamera and tearDownCamera)
 */
	virtual cv::Mat retrieveFrame() = 0;



};
} /* namespace video */
} /* namespace augmentarium */

#endif /* MODULES_VIDEO_FRAMEPROCESSOR_H_ */
