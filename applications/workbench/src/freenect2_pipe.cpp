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

//std
#include <stdexcept>

namespace reco {
namespace workbench {

// kinect v2 resolution
const unsigned int freenect2_pipe::depth_image_width = 512;
const unsigned int freenect2_pipe::depth_image_height = 424;
const unsigned int freenect2_pipe::rgb_image_width = 1920;
const unsigned int freenect2_pipe::rgb_image_height = 1080;
const unsigned int freenect2_pipe::num_channels_per_feed = 2;
const unsigned int freenect2_pipe::depth_channel_offset = 1;
const unsigned int freenect2_pipe::rgb_channel_offset = 0;

/*
 * Set camera using requested URI
 */
void freenect2_pipe::set_camera(const std::string& cam_uri) {
	rgbd_camera = hal::Camera(cam_uri);
	has_camera = true;

	const size_t num_channels = rgbd_camera.NumChannels();

	//check that we have appropriate number of channels
	//the total number of channels must be evenly divisible by the number of channels per feed
	if (num_channels % freenect2_pipe::num_channels_per_feed != 0) {
		err(std::invalid_argument)
				<< "Incorrect number of channels for Kinect v2 feed! Need exactly "
				<< freenect2_pipe::num_channels_per_feed << "! Please check cam uri. Got " << cam_uri
				<< enderr;
	}

	num_kinects = num_channels / freenect2_pipe::num_channels_per_feed;

	//check that the feed sizes match for RGB & depth, for each kinect
	if (rgbd_camera.Width(freenect2_pipe::rgb_channel_offset) != freenect2_pipe::rgb_image_width
			||
			rgbd_camera.Height(freenect2_pipe::rgb_channel_offset)
					!= freenect2_pipe::rgb_image_height
			||
			rgbd_camera.Width(freenect2_pipe::depth_channel_offset)
					!= freenect2_pipe::depth_image_width
			||
			rgbd_camera.Height(freenect2_pipe::depth_channel_offset)
					!= freenect2_pipe::depth_image_height
					) {
		err(std::invalid_argument) << "Wrong camera dimensions. " << std::endl
				<< "Expecting (width x height): " << std::endl
				<< "  RGB: " << freenect2_pipe::rgb_image_width << " x "
				<< freenect2_pipe::rgb_image_height << std::endl
				<< "  depth: " << freenect2_pipe::depth_image_width << " x "
				<< freenect2_pipe::depth_image_height << std::endl
				<< "Got:" << std::endl
				<< "  RGB: " << rgbd_camera.Width(freenect2_pipe::rgb_channel_offset) << " x "
				<< rgbd_camera.Height(freenect2_pipe::rgb_channel_offset) << std::endl
				<< "  depth: " << rgbd_camera.Width(freenect2_pipe::depth_channel_offset) << " x "
				<< rgbd_camera.Height(freenect2_pipe::depth_channel_offset) << std::endl
				<< "Got cam_uri: " << cam_uri << enderr;
	}

}

freenect2_pipe::freenect2_pipe(kinect2_data_source source, const std::string& path) {
	switch (source) {
	case kinect2_data_source::kinect2_device:
		//note: will open all kinects as "single" camera
		set_camera("freenect2:[rgb=1,ir=0,depth=1]//");
		break;
	case kinect2_data_source::hal_log:
		set_camera("log://" + path);
		break;
	case kinect2_data_source::image_files:
		//TODO: not sure this is the right way to read image files
		set_camera("file");
		break;
	default:
		err(std::invalid_argument) << "Unknown value for kinect2_data_source. Got: " << source
				<< enderr;
		break;
	}

}

freenect2_pipe::~freenect2_pipe() {

}

void freenect2_pipe::run() {
	while (!stop_requested) {

	}
}

} /* namespace workbench */
} /* namespace reco */
