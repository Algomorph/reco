/*
 * pipe.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/datapipe/pipe.h>

namespace reco {
namespace datapipe {

pipe::pipe(frame_buffer_type buffer, int num_channels):
		num_channels(num_channels),
		buffer(buffer){}

pipe::~pipe(){}


/**
 * @return number of source's kinect devices.
 */
int pipe::get_num_channels() {
	return num_channels;
}

/**
 * @return a shared pointer to the buffer
 */
frame_buffer_type pipe::get_buffer() {
	return this->buffer;
}

} /* namespace datapipe */
} /* namespace reco */
