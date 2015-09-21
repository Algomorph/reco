/*
 * freenect2_pipe.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//datapipe
#include <reco/datapipe/freenect2_pipe_old.h>
#include <reco/datapipe/kinect_v2_info.h>

//utils
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>

//std
#include <stdexcept>
#include <memory>
#include <unistd.h>

//opencv

#include <opencv2/core/types_c.h>

namespace reco {
namespace datapipe {

freenect2_pipe_old::freenect2_pipe_old(buffer_type buffer,
		kinect2_data_source source, const std::string& path) :
				stop_requested(false),
				buffer(buffer),
				runner_thread(&freenect2_pipe_old::run, this)

{
	switch (source) {
	case kinect2_data_source::kinect2_device:
		//note: will open all kinects as "single" camera
		set_camera("freenect2:[rgb=1,ir=0,depth=1]//");
		break;
	case kinect2_data_source::hal_log:
		set_camera("log://" + path);
		break;
	case kinect2_data_source::image_folder:
		//TODO: implementation pending. not sure if "set_camera("file")" is the right way to read image files
		throw reco::utils::not_implemented();
		break;
	default:
		err(std::invalid_argument) << "Unknown value for kinect2_data_source. Got: " << source
				<< enderr;
		break;
	}
}

freenect2_pipe_old::~freenect2_pipe_old() {
}

static void check_channel_dimensions(const hal::Camera& rgbd_camera, const std::string& cam_uri,
		int ix_channel, int channel_offset) {
	if ((int) rgbd_camera.Width(ix_channel) != kinect_v2_info::channels[channel_offset]->width()
			||
			(int) rgbd_camera.Height(ix_channel)
					!= kinect_v2_info::channels[channel_offset]->height()) {
		err(std::invalid_argument) << "Wrong camera dimensions. " << std::endl
				<< "Expecting (width x height) for "
				<< kinect_v2_info::channels[channel_offset]->name() << ":" << std::endl
				<< kinect_v2_info::channels[channel_offset]->width() << " x "
				<< kinect_v2_info::channels[channel_offset]->height() << std::endl
				<< "Got dimensions (width x height):" << std::endl
				<< rgbd_camera.Width(ix_channel) << " x "
				<< rgbd_camera.Height(ix_channel) << std::endl
				<< "Got cam_uri: " << cam_uri
				<< enderr;
	}
}

/*
 * Set camera using requested URI
 */
void freenect2_pipe_old::set_camera(const std::string& cam_uri) {
	rgbd_camera = hal::Camera(cam_uri);
	has_camera = true;
	num_channels = (int) rgbd_camera.NumChannels();
	//check that we have appropriate number of channels
	//the total number of channels must be evenly divisible by the number of channels per feed
	if (num_channels % kinect_v2_info::channels.size() != 0) {
		err(std::invalid_argument)
		<< "Incorrect number of channels for a set of Kinect v2 feeds! Need a multiple of "
				<< kinect_v2_info::channels.size() << "! Please check cam uri. Current uri: "
				<< cam_uri
				<< enderr;
	}
	num_kinects = num_channels / kinect_v2_info::channels.size();
	//check that the feed sizes match for RGB & depth, for each kinect
	for (int ix_channel; ix_channel < num_channels; ix_channel++) {
		check_channel_dimensions(rgbd_camera, cam_uri, ix_channel,
				ix_channel % kinect_v2_info::channels.size());
	}
}

int freenect2_pipe_old::get_num_kinects() {
	return num_kinects;
}

int freenect2_pipe_old::get_num_channels() {
	return num_channels;
}

freenect2_pipe_old::buffer_type freenect2_pipe_old::get_buffer() {
	return this->buffer;
}

void freenect2_pipe_old::run() {
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
				&& rgbd_camera.Capture(*images)) {
			this->buffer->push_back(images);

			emit frame();
			images = hal::ImageArray::Create();
		}
	}
	rgbd_camera.Clear();
}

void freenect2_pipe_old::pause() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = false;
}

void freenect2_pipe_old::play() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = true;
	pause_cv.notify_one();

}


void freenect2_pipe_old::stop() {
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
