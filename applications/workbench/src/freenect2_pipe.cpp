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
	if (has_camera) {
		for (void* data_part : data) {
			free(data_part);
		}
	}
}

/*
 * Set camera using requested URI
 */
void freenect2_pipe::set_camera(const std::string& cam_uri) {
	rgbd_camera = hal::Camera(cam_uri);
	has_camera = true;

	const size_t num_channels = rgbd_camera.NumChannels();

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

	//allocate memory for output
	size_t image_set_size = num_kinects *
			(kinect_v2_info::rgb_image_size + kinect_v2_info::depth_image_size);
	for (int i_data_part = 0; i_data_part < 2; i_data_part++) {
		data[i_data_part] = (uchar*)malloc(image_set_size);
	}

}

/**
 * Get number of kinect feeds
 * @return number of kinect feeds in the source, 0 if no source is available
 */
uint freenect2_pipe::get_num_kinects() {
	return num_kinects;
}

void freenect2_pipe::run() {

	{
		std::unique_lock<std::mutex> lk(this->pause_mtx);
		std::cout << "heya, playback allowed: " << playback_allowed << std::endl;
		pause_cv.wait(lk, [&] {return playback_allowed;});
	}
	if (!has_camera) {
		std::cout << "No camera connected..." << std::endl;
		return;
	}



	const int num_channels = this->rgbd_camera.NumChannels();
	std::vector<cv::Mat> mats0(num_channels);
	std::vector<cv::Mat> mats1(num_channels);
	std::array<std::vector<cv::Mat>, 2> mats = { mats0, mats1 };
	int offset = 0;

	for (int i_mat = 0; i_mat < num_channels; i_mat += 2) {
		for (std::vector<cv::Mat> mat_vec : mats) {
			mat_vec[i_mat] = cv::Mat(kinect_v2_info::rgb_image_height,
					kinect_v2_info::rgb_image_width, CV_8UC3, data[0] + offset);
			mat_vec[i_mat + 1] = cv::Mat(kinect_v2_info::rgb_image_height,
					kinect_v2_info::rgb_image_width, CV_8UC3,
					data[0] + offset + kinect_v2_info::rgb_image_size);
		}
		offset += (kinect_v2_info::rgb_image_size + kinect_v2_info::depth_image_size);
	}

	int ix = 0;
	std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
	while (playback_allowed
			&& rgbd_camera.Capture(*images)) {

		std::unique_ptr<hal::CameraMsg> pCameraMsg(new hal::CameraMsg);

		this->buffer->push_back(images);

		emit frame();
		images = hal::ImageArray::Create();
		ix = (ix + 1) % 2;
	}
}

/**
 * Pause playback
 */
void freenect2_pipe::pause() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = false;
}

/**
 * Start/resume playback
 */
void freenect2_pipe::play() {
	std::unique_lock<std::mutex> lk(this->pause_mtx);
	playback_allowed = true;
	pause_cv.notify_one();

}

} /* namespace workbench */
} /* namespace reco */
