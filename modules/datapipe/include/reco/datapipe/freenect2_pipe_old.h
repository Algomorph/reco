/*
 * freenect2_pipe.h
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_FREENECT2_PIPE_H
#define RECO_DATAPIPE_FREENECT2_PIPE_H_

//qt
#include <QObject>

//opencv
#include <opencv2/core/core.hpp>

//utils
#include <reco/utils/swap_buffer.h>

//arpg
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/CameraDevice.h>

//std
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace reco {
namespace datapipe {


/**
 * @brief Directs one or more kinect feeds from an ARPG HAL-based source to apps that process and output the data
 * Object for retrieving kinect data (from "somewhere" in HAL) and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
class freenect2_pipe_old : public QObject {
Q_OBJECT

public:
/**
 * Type of the buffer object required to use the pipe
 */
	typedef std::shared_ptr<utils::queue<std::shared_ptr<hal::ImageArray>>> buffer_type;

protected:
	bool playback_allowed = false;//start out paused
	bool stop_requested = false;
	std::condition_variable pause_cv;
	std::mutex pause_mtx;
	void run();

private:
	hal::Camera rgbd_camera;
	bool has_camera = false;
	int num_kinects = 0;
	int num_channels = 0;

	void set_camera(const std::string& cam_uri);
	buffer_type buffer;
	std::thread runner_thread;


public:

	enum kinect2_data_source {
		hal_log, kinect2_device, image_folder //TODO: implement image folder support later if needed
	};

	/**
	 * @brief Primary constructor
	 * Constructs the object with the specified buffer and initializes the retrieval based on the kinect2 data source
	 * @param buffer a thread-safe buffer object for storing the output RGB & Depth feeds
	 * @param source type of the input source
	 * @param path necessary when the source is either an ARPG HAL log (in which case, represents path to the log file) or a file folder (in which case, represents the directory)
	 */
	freenect2_pipe_old(buffer_type buffer,
			kinect2_data_source source = hal_log, const std::string& path = "capture.log");
	virtual ~freenect2_pipe_old();
	/**
	 * @return number of source's kinect devices.
	 */
	int get_num_kinects();

	/**
	 * @return number of source's kinect devices.
	 */
	int get_num_channels();


	/**
	 * @return a shared pointer to the buffer
	 */
	buffer_type get_buffer();

public slots:
	/**
	 * Shuts down the pipe & feed for good.
	 * Intended to be called at the end of the program's execution.
	 * @note Can only be called once after objects' creation. Afterward, the object is unusable.
	 */
	void stop();

	/**
	 * Pause playback
	 */
	void pause();

	/**
	 * Start/resume playback
	 */
	void play();

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
	void frame();

};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_SRC_FREENECT2_PIPE_H_ */
