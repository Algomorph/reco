/*
 * Freenect2Connector.cpp
 *
 *  Created on: May 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//freenect2
#include <reco/datapipe/freenect2_connector.h>
#include <libfreenect2/frame_listener_impl.h>

//std
#include <exception>

namespace reco {
namespace datapipe {

void freenect2_connector::init(bool capture_RGB, bool capture_depth, bool capture_IR) {
	if (!this->device) {
		throw std::runtime_error(
				"Failed to open Kinect2 device. Consult previous stdout for clues.");
	}

	using namespace libfreenect2;
	unsigned int types = 0;
	if (capture_RGB)
		types |= libfreenect2::Frame::Color;
	if (capture_IR)
		types |= libfreenect2::Frame::Ir;
	if (capture_depth)
		types |= libfreenect2::Frame::Depth;
	if (types == 0) {
		throw std::runtime_error("Freenect2: no channel given (rgb, ir, depth)");
	}

	listener = std::shared_ptr<SyncMultiFrameListener>(types);
	if(capture_RGB){
		device->setColorFrameListener(listener.get());
	}
	if(capture_IR || capture_depth){
		device->setIrAndDepthFrameListener(listener.get());
	}

}

freenect2_connector::freenect2_connector(int index, bool captureRGB, bool capture_depth,
		bool capture_IR) {
	this->device = std::shared_ptr<libfreenect2::Freenect2Device>(freenect2.openDevice(index));
	init(captureRGB, capture_depth, capture_IR);

}

freenect2_connector::freenect2_connector(std::string serial_number, bool captureRGB, bool capture_depth,
		bool capture_IR) {
	this->device = std::shared_ptr<libfreenect2::Freenect2Device>(freenect2.openDevice(serial_number));
	init(captureRGB, capture_depth, capture_IR);

}

freenect2_connector::~freenect2_connector() {
	// TODO Auto-generated destructor stub
}

} //end namespace reco
} //end namespace datapipe

