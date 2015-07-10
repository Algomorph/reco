/*
 * freenect2_pipe.h
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */


#ifndef MODULES_DATAPIPE_SRC_FREENECT2_PIPE2_H_
#define MODULES_DATAPIPE_SRC_FREENECT2_PIPE2_H_
#pragma once

//qt
#include <QObject>
//opencv
#include <opencv2/core/core.hpp>

//datapipe
#include <reco/datapipe/thread_runnable.h>

//utils
#include <reco/utils/swap_buffer.h>

//arpg
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/CameraDevice.h>

//std
#include <vector>
#include <thread>

namespace reco {
namespace workbench {

/**
 * Object for retrieving kinect data (from "somewhere" in HAL) and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
/*
 * TODO: integrate HAL interface
 */
class freenect2_pipe2:
		public datapipe::thread_runnable {
Q_OBJECT

private:
	hal::Camera rgbd_camera;
	bool has_camera = false;
	uint num_kinects = 0;
	std::shared_ptr<utils::swap_buffer<std::vector<cv::Mat>>> buffer;

	void set_camera(const std::string& cam_uri);

public:

	enum kinect2_data_source {
		hal_log, kinect2_device, image_folder //TODO: implement image folder support later if needed
	};

	freenect2_pipe2(std::shared_ptr<utils::swap_buffer<std::vector<cv::Mat>>>,
			kinect2_data_source source = hal_log, const std::string& path = "capture.log");
	virtual ~freenect2_pipe2();
	uint get_num_kinects();


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
	 * and put into the buffer
	 * @param
	 */
	void frame();
	void rgb_frame(cv::Mat image);
	void output_ready();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_FREENECT2_PIPE_H_ */
