/*
 * FrameProcessor.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_FRAMEPROCESSOR_H_
#define RECO_DATAPIPE_FRAMEPROCESSOR_H_

#pragma once

//OpenCV
#include <opencv2/core/core.hpp>
#include <QObject>
//local
#include "static_type_registration.h"

//standard
#include <exception>
#include <memory>


namespace reco {
namespace datapipe {

/**
 * Captures and each subsequent frame of the video
 */


class video_source:
		public QObject {
Q_OBJECT
protected:
	cv::Mat frame;

public:
	video_source() {}
	virtual ~video_source() {}


	/**
	 * Get frame width
	 */
	virtual unsigned get_width() = 0;
	/**
	 * Get frame height
	 */
	virtual unsigned get_height() = 0;
	/**
	 * Refresh/update the current frame
	 */
	virtual bool capture_frame() = 0;

	/**
	 * Prepare camera for capture or video source for playback
	 */
	virtual bool set_up() = 0;
	/**
	 * Stop the capture, release camera / deallocate video source
	 */
	virtual void tear_down() = 0;
	/**
	 * Retrieve the current frame (assumes being called between setUpCamera and tearDownCamera)
	 */
	cv::Mat copy_frame(){
		return this->frame.clone();
	}
	/**
	 * Retrieve the current frame (assumes being called between setUpCamera and tearDownCamera)
	 */
	cv::Mat take_frame(){
		return this->frame;
	}

};
} /* namespace reco */
} /* namespace datapipe */

#endif /* RECO_DATAPIPE_FRAMEPROCESSOR_H_ */
