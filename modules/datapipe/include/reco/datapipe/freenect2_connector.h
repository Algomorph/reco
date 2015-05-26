/*
 * Freenect2Connector.h
 *
 *  Created on: May 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_DATAPIPE_FREENECT2CONNECTOR_H_
#define RECO_DATAPIPE_FREENECT2CONNECTOR_H_

//std
#include <memory>
#include <string>

//freenect
#include <libfreenect2/libfreenect2.hpp>

namespace reco {
namespace datapipe {

/**
 * A wrapper around the freenect2 driver, intended to abstract some of the video-piping details
 */
class freenect2_connector {
private:
	libfreenect2::Freenect2 freenect2;
	int index;
	std::shared_ptr<libfreenect2::Freenect2Device> device;
	std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener;
	void init(bool captureRGB, bool captureDepth, bool captureIR);
public:
	freenect2_connector(int index = 0, bool capture_RGB = true, bool capture_depth = true,
			bool capture_IR = false);
	freenect2_connector(std::string serial_number, bool capture_RGB = true, bool capture_depth = true,
			bool capture_IR = false);
	virtual ~freenect2_connector();
};

} //end namespace datapipe
} //end namespace reco

#endif /*RECO_DATAPIPE_FREENECT2CONNECTOR_H_ */
