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

namespace reco {
namespace workbench {

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
				<< kinect_v2_info::num_channels_per_feed << "! Please check cam uri. Got " << cam_uri
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

freenect2_pipe::freenect2_pipe(kinect2_data_source source, const std::string& path) {
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
/**
 * Get number of kinect feeds
 * @return number of kinect feeds in the source, 0 if no source is available
 */
uint freenect2_pipe::get_num_kinects(){
	return num_kinects;
}

void freenect2_pipe::run() {
	if(has_camera){
		std::vector<cv::Mat> images;
		while (!stop_requested && !pause_requested && rgbd_camera.Capture(images)) {
			emit frame(images);
		}
	}
}

} /* namespace workbench */
} /* namespace reco */
