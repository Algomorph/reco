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

//arpg
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/CameraDevice.h>

//std
#include <vector>

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
	bool has_camera = false;
	uint num_kinects = 0;
	std::shared_ptr<std::vector<cv::Mat>> images;

	void set_camera(const std::string& cam_uri);

public:

	enum kinect2_data_source {
		hal_log, kinect2_device, image_folder //TODO: implement image folder support later if needed
	};

	freenect2_pipe(kinect2_data_source source = hal_log, const std::string& path = "capture.log");
	virtual ~freenect2_pipe();
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
	 * @param
	 */
	void frame(std::vector<cv::Mat> images);
	void output_ready();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* MODULES_DATAPIPE_SRC_FREENECT2_PIPE_H_ */
