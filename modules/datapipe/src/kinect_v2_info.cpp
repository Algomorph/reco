/*
 * kinect_v2_info.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#include <reco/datapipe/kinect_v2_info.h>

namespace reco {
namespace datapipe {

//TODO: avoid magic numbers for channel offset via static const functions
enum channel_offset{
		RGB = 0,
		DEPTH = 1
	};
const data_channel kinect_v2_info::rgb_channel("RGB channel",1920,1080,0,3);
const data_channel kinect_v2_info::depth_channel("depth channel",512,424,1,4);
const std::array<const data_channel*,2> kinect_v2_info::channels({&rgb_channel,&depth_channel});

//inaccessible
kinect_v2_info::kinect_v2_info()
{}

kinect_v2_info::~kinect_v2_info()
{}

} /* namespace datapipe */
} /* namespace reco */
