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

multichannel_pipe::multichannel_pipe(buffer_type buffer):
	buffer(buffer),
	playback_allowed(false),
	stop_requested(false)
	{

}

multichannel_pipe::~multichannel_pipe() {

}


/**
 * Set camera using requested URI
 * @param[in] cam_uri the requested camera uri
 */
void multichannel_pipe::set_camera(const std::string& cam_uri) {
	camera = hal::Camera(cam_uri);
	has_camera = true;
	num_channels = (int) camera.NumChannels();
	//check that we have appropriate number of channels
	//i.e. for multip feeds, the total number of channels must be evenly divisible
	//by the number of channels per feed
	check_channel_number(cam_uri, num_channels);

	//check that the feed sizes match for RGB & depth, for each kinect
	for (int ix_channel; ix_channel < num_channels; ix_channel++) {
		check_channel_dimensions(cam_uri, ix_channel);
	}
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
		if (!has_camera) {
			err(std::runtime_error)
			<< "freenect2_pipe::run(): failed to run because there is no camera connected.";
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
