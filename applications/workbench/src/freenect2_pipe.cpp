/*
 * freenect2_pipe.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <src/freenect2_pipe.h>
#include <reco/utils/cpp_exception_util.h>
#include <reco/workbench/kinect_v2_info.h>

//std
#include <stdexcept>
#include <memory>
#include <unistd.h>

//opencv

#include <opencv2/core/types_c.h>

#define SWAP_BIN_IX(ix) (ix = (ix + 1) % 2);

namespace reco {
namespace workbench {



freenect2_pipe::freenect2_pipe(buffer_type buffer,
		kinect2_data_source source, const std::string& path) :
				buffer(buffer),
				runner_thread(&freenect2_pipe::run, this)
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

freenect2_pipe::~freenect2_pipe() {
}

/*
 * Set camera using requested URI
 */
void freenect2_pipe::set_camera(const std::string& cam_uri) {
	rgbd_camera = hal::Camera(cam_uri);
	has_camera = true;

	num_channels = (int)rgbd_camera.NumChannels();

	//check that we have appropriate number of channels
	//the total number of channels must be evenly divisible by the number of channels per feed
	if (num_channels % kinect_v2_info::num_channels_per_feed != 0) {
		err(std::invalid_argument)
		<< "Incorrect number of channels for Kinect v2 feed! Need exactly "
				<< kinect_v2_info::num_channels_per_feed << "! Please check cam uri. Got "
				<< cam_uri
				<< enderr;
	}

	num_kinects = num_channels / kinect_v2_info::num_channels_per_feed;

	//check that the feed sizes match for RGB & depth, for each kinect
	if (rgbd_camera.Width(kinect_v2_info::rgb_channel_offset) != kinect_v2_info::rgb_image_width
			||
			rgbd_camera.Height(kinect_v2_info::rgb_channel_offset)
					!= kinect_v2_info::rgb_image_height
			||
			rgbd_camera.Width(kinect_v2_info::depth_channel_offset)
					!= kinect_v2_info::depth_image_width
			||
			rgbd_camera.Height(kinect_v2_info::depth_channel_offset)
					!= kinect_v2_info::depth_image_height
					) {
		err(std::invalid_argument) << "Wrong camera dimensions. " << std::endl
				<< "Expecting (width x height): " << std::endl
				<< "  RGB: " << kinect_v2_info::rgb_image_width << " x "
				<< kinect_v2_info::rgb_image_height << std::endl
				<< "  depth: " << kinect_v2_info::depth_image_width << " x "
				<< kinect_v2_info::depth_image_height << std::endl
				<< "Got:" << std::endl
				<< "  RGB: " << rgbd_camera.Width(kinect_v2_info::rgb_channel_offset) << " x "
				<< rgbd_camera.Height(kinect_v2_info::rgb_channel_offset) << std::endl
				<< "  depth: " << rgbd_camera.Width(kinect_v2_info::depth_channel_offset) << " x "
				<< rgbd_camera.Height(kinect_v2_info::depth_channel_offset) << std::endl
				<< "Got casm_uri: " << cam_uri << enderr;
	}

}


int freenect2_pipe::get_num_kinects() {
	return num_kinects;
}

int freenect2_pipe::get_num_channels() {
	return num_channels;
}

freenect2_pipe::buffer_type freenect2_pipe::get_buffer(){
	return this->buffer;
}

void freenect2_pipe::run() {

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

}


void freenect2_pipe::pause() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = false;
}

void freenect2_pipe::play() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = true;
	pause_cv.notify_one();

}

void freenect2_pipe::join_thread() {
	this->rgbd_camera.Clear();
	this->runner_thread.join();
}

void freenect2_pipe::stop() {
	{
		std::unique_lock<std::mutex> lk(this->pause_mtx);
		if (!playback_allowed) {
			playback_allowed = true;
			pause_cv.notify_one();
		}
	}
	stop_requested = true;

}

} /* namespace workbench */
} /* namespace reco */
