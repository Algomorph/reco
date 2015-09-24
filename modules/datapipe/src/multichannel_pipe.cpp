/*
 * multichannel_pipe.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/multichannel_pipe.h>

//utils
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>

namespace reco {
namespace datapipe {

multichannel_pipe::multichannel_pipe(buffer_type buffer, std::string camera_uri):
	buffer(buffer),
	playback_allowed(false),
	stop_requested(false),
	runner_thread(&multichannel_pipe::work, this)
	{
	set_camera(camera_uri);
}

multichannel_pipe::~multichannel_pipe() {
	this->stop();
}



/**
 * Set camera using requested URI
 * @param[in] cam_uri the requested camera uri
 */
void multichannel_pipe::set_camera(const std::string& cam_uri) {
	puts(cam_uri);
	camera = hal::Camera(cam_uri);
	num_channels = (int) camera.NumChannels();

}

/**
 * @return number of source's kinect devices.
 */
int multichannel_pipe::get_num_channels() {
	return num_channels;
}

/**
 * @return a shared pointer to the buffer
 */
multichannel_pipe::buffer_type multichannel_pipe::get_buffer() {
	return this->buffer;
}

void multichannel_pipe::work() {
	while (!stop_requested) {
		{
			std::unique_lock<std::mutex> lk(this->pause_mtx);
			pause_cv.wait(lk, [&] {return playback_allowed;});
		}
		if (camera.Empty() && !stop_requested) {
			err(std::runtime_error)
			<< "Failed to run pipe because there is no camera connected.";
			return;
		}
		std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
		while (!stop_requested && playback_allowed
				&& camera.Capture(*images)) {
			this->buffer->push_back(images);

			emit frame();
			images = hal::ImageArray::Create();
		}
	}

	camera.Clear();
}


/**
 * Start/resume playback
 */
void multichannel_pipe::run() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = true;
	pause_cv.notify_one();
}


/**
 * Pause playback
 */
void multichannel_pipe::pause() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = false;
}

/**
 * Shuts down the pipe & feed for good.
 * Intended to be called at the end of the program's execution.
 * @note Can only be called once after objects' creation. Afterward, the object is unusable.
 */
void multichannel_pipe::stop() {
	stop_requested = true;
	buffer->clear();
	{
		std::unique_lock<std::mutex> lk(this->pause_mtx);
		//if the feed is paused, unpause it
		if (!playback_allowed) {
			playback_allowed = true;
			pause_cv.notify_one();
		}
	}
	this->runner_thread.join();

}

} /* namespace datapipe */
} /* namespace reco */
