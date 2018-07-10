/*
 * multichannel_pipe.h
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

//qt
#include <QObject>

//arpg
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/CameraDevice.h>

//standard
#include <memory>
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>

//utils
#include <reco/utils/queue.h>

//datapipe
#include <reco/datapipe/typedefs.h>
#include <reco/datapipe/pipe.h>

namespace reco {
namespace datapipe {


/**
 * @brief Directs one or more video feeds from an ARPG HAL-based source to apps that process and output the data
 * Object for retrieving video data (from "somewhere" in HAL) and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
class hal_pipe:
		public pipe{

Q_OBJECT

public:
/**
 * Type of the buffer object required to use the pipe
 */
	hal_pipe(frame_buffer_type buffer, std::string camera_uri);
	virtual ~hal_pipe();

protected:
	std::string camera_uri;
	hal::Camera camera;
private:
	void initialize();

	//thread state
	bool playback_allowed;
	bool stop_requested;
	std::thread runner_thread;
	std::condition_variable pause_cv;
	std::mutex pause_mtx;

	void work();

public slots:

	void stop();
	void pause();
	void run();
};

} /* namespace datapipe */
} /* namespace reco */
