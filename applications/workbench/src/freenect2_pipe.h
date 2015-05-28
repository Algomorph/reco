/*
 * freenect2_pipe.h
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef MODULES_DATAPIPE_SRC_FREENECT2_PIPE_H_
#define MODULES_DATAPIPE_SRC_FREENECT2_PIPE_H_

//qt
#include <QObject>
//opencv
#include <opencv2/core/core.hpp>

//datapipe
#include <reco/datapipe/runnable.h>

namespace reco {
namespace workbench {

/**
 * Object for retrieving kinect data (from "somewhere") and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
class freenect2_pipe:public datapipe::runnable {
	Q_OBJECT


public:

	freenect2_pipe();
	virtual ~freenect2_pipe();

protected:

	virtual void run();


signals:
/**
 * Emitted on error
 * @param error
 */
	void error(QString err);
	/**
	 * Emitted when a new frame had been processed
	 * @param
	 */
	void color_frame_ready(cv::Mat);
	void depth_frame_ready(cv::Mat);
	void output_ready();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_FREENECT2_PIPE_H_ */
