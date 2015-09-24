/*
 * multichannel_pipe.h
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_MULTICHANNEL_PIPE_H_
#define RECO_DATAPIPE_MULTICHANNEL_PIPE_H_

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

namespace reco {
namespace datapipe {


/**
 * @brief Directs one or more video feeds from an ARPG HAL-based source to apps that process and output the data
 * Object for retrieving video data (from "somewhere" in HAL) and pushing it off to various downstream actors,
 * such as display node(s), recording/storing node(s), and/or processing node(s).
 */
class multichannel_pipe:
		public QObject{

Q_OBJECT

public:
/**
 * Type of the buffer object required to use the pipe
 */
	/*TODO 354 put this typedef into a separate include file within datapipe.
	 * It seems like everyone and his uncle are using it, no need to redefine it every time.*/
	typedef std::shared_ptr<utils::queue<std::shared_ptr<hal::ImageArray>>> buffer_type;
	multichannel_pipe(buffer_type buffer, std::string camera_uri);
	virtual ~multichannel_pipe();

	int get_num_channels();
	buffer_type get_buffer();

protected:

	hal::Camera camera;

	void set_camera(const std::string& cam_uri);
	//Called during set_camera
	virtual void check_channel_number(const std::string& cam_uri, size_t num_channels) = 0;
	virtual void check_channel_dimensions(const std::string& cam_uri, int ix_channel) = 0;


	int num_channels = 0;

private:
	void initialize();
	buffer_type buffer;

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

signals:
	/**
	 * Emitted on error
	 * @param error
	 */
	void error(QString err);
	/**
	 * Emitted when a new frame had been processed and pushed onto the buffer
	 */
	void frame();
};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_MULTICHANNEL_PIPE_H_ */
