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

//HAL
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/CameraDevice.h>

namespace reco {
namespace workbench {

/**
 * Object for retrieving kinect data (from "somewhere" in HAL) and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
/*
 * TODO: integrate HAL interface
 */
class freenect2_pipe:
		public datapipe::runnable {
Q_OBJECT

private:
	hal::Camera rgbd_camera;
	bool has_camera;
	int num_kinects;

	void set_camera(const std::string& cam_uri);

public:
	static const unsigned int rgb_image_width;
	static const unsigned int rgb_image_height;
	static const unsigned int depth_image_width;
	static const unsigned int depth_image_height;
	static const unsigned int num_channels_per_feed;
	static const unsigned int depth_channel_offset;
	static const unsigned int rgb_channel_offset;

	enum kinect2_data_source {
		hal_log, kinect2_device, image_files //TODO: implement image file support later if needed
	};

	freenect2_pipe(kinect2_data_source source, const std::string& path = "capture.log");
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
