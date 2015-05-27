/*
 * Freenect2Connector.cpp
 *
 *  Created on: May 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//freenect2
#include <reco/datapipe/freenect2_connector.h>


//std
#include <exception>

namespace reco {
namespace datapipe {

/*
 * helper initialization function, called by constructors
 */
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

	listener = std::make_shared<SyncMultiFrameListener>(types);
	if (capture_RGB) {
		device->setColorFrameListener(listener.get());
	}
	if (capture_IR || capture_depth) {
		device->setIrAndDepthFrameListener(listener.get());
	}
}

/**
 * Construct a connector using index of the connected Kinect2 device, if such device is available.
 * There are no guarantees that the index is preserved is the configuration of the attached Kinect2s is altered.
 * One or more of {capture_RGB, capture_depth, capture_IR} has to be set to true.
 * @param index index of the kinect2 to connect to.
 * @param capture_RGB whether to capture RGB stream or not.
 * @param capture_depth whether to capture depth stream or not.
 * @param capture_IR whether to capture the IR stream or not.
 */
freenect2_connector::freenect2_connector(int index, bool capture_RGB, bool capture_depth,
		bool capture_IR) :
		capture_RGB(capture_RGB), capture_depth(capture_depth), capture_IR(capture_IR),
		device_is_running(false){
	this->device = std::shared_ptr<libfreenect2::Freenect2Device>(freenect2.openDevice(index));
	init(capture_RGB, capture_depth, capture_IR);
}

/**
 * Construct a connector using serial number of the connected Kinect2 device, if such device is available.
 * Serial numbers are guaranteed to be unique and persistent for each device.
 * One or more of {capture_RGB, capture_depth, capture_IR} has to be set to true.
 * @param index index of the kinect2 to connect to.
 * @param capture_RGB whether to capture RGB stream or not.
 * @param capture_depth whether to capture depth stream or not.
 * @param capture_IR whether to capture the IR stream or not.
 */
freenect2_connector::freenect2_connector(std::string serial_number, bool capture_RGB,
		bool capture_depth, bool capture_IR) :
		capture_RGB(capture_RGB), capture_depth(capture_depth), capture_IR(capture_IR),
		device_is_running(false){
	this->device = std::shared_ptr<libfreenect2::Freenect2Device>(
			freenect2.openDevice(serial_number));
	init(capture_RGB, capture_depth, capture_IR);
}

/*
 * Desctructor closes the device.
 */
freenect2_connector::~freenect2_connector() {
	if(device_is_running){
		device->stop();
	}
	device->close();
}

} //end namespace reco
} //end namespace datapipe

