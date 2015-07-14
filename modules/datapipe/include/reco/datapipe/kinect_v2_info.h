/*
 * kinect_v2_info.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_DATAPIPE_KINECT_V2_INFO_H_
#define RECO_DATAPIPE_KINECT_V2_INFO_H_
#pragma once

//datapipe
#include <reco/datapipe/data_channel.h>

//Standard Library
#include <array>

namespace reco {
namespace datapipe {

class kinect_v2_info {

private:
	kinect_v2_info();
public:

	static const data_channel depth_channel;
	static const data_channel rgb_channel;

	static const std::array<const data_channel*,2> channels;

	virtual ~kinect_v2_info();
};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_SRC_KINECT_V2_INFO_H_ */
