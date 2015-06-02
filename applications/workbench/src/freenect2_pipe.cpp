/*
 * freenect2_pipe.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <src/freenect2_pipe.h>



//std
#include <stdexcept>

namespace reco {
namespace workbench {

// values given by libfreenect
const unsigned int freenect2_pipe::DEPTH_IMAGE_WIDTH = 512;
const unsigned int freenect2_pipe::DEPTH_IMAGE_HEIGHT = 424;

/*
 * Set camera using requested URI
 */
void freenect2_pipe::set_camera(const std::string& cam_uri) {
	rgbd_camera = hal::Camera(cam_uri);
	has_camera = true;

	const size_t num_channels = rgbd_camera.NumChannels();


	//if()

	for (size_t ii = 0; ii < num_channels; ++ii) {
		std::cout << "\t" << rgbd_camera.Width(ii) << "x" << rgbd_camera.Height(ii)
				<< std::endl;
	}
}

freenect2_pipe::freenect2_pipe(kinect2_data_source) {

}

freenect2_pipe::~freenect2_pipe() {

}

void freenect2_pipe::run() {
	while (!stop_requested) {

	}
}

} /* namespace workbench */
} /* namespace reco */
