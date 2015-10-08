/*
 * multichannel_pipe.cpp
 *
 *  Created on: Sep 21, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/hal_pipe.h>
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>

namespace reco {
namespace datapipe {

hal_pipe::hal_pipe(frame_buffer_type buffer, std::string camera_uri):
	pipe(buffer),
	camera_uri(camera_uri),
	camera(hal::Camera(camera_uri)),
	playback_allowed(false),
	stop_requested(false),
	runner_thread(&hal_pipe::work, this)
	{
	num_channels = (int)camera.NumChannels();
}

hal_pipe::~hal_pipe() {
	this->stop();
}

void hal_pipe::work() {
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
void hal_pipe::run() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = true;
	pause_cv.notify_one();
}


/**
 * Pause playback
 */
void hal_pipe::pause() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = false;
}

/**
 * Shuts down the pipe & feed for good.
 * Intended to be called at the end of the program's execution.
 * @note Can only be called once after objects' creation. Afterward, the object is unusable.
 */
void hal_pipe::stop() {
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
	if(this->runner_thread.joinable()){
		this->runner_thread.join();
	}

}

} /* namespace datapipe */
} /* namespace reco */
