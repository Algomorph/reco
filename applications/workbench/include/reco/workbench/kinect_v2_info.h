/*
 * kinect_v2_info.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_WORKBENCH_KINECT_V2_INFO_H_
#define RECO_WORKBENCH_KINECT_V2_INFO_H_
#pragma once

namespace reco {
namespace workbench {

class kinect_v2_info {

private:
	kinect_v2_info();
public:
	static const unsigned int rgb_image_width;
	static const unsigned int rgb_image_height;
	static const unsigned int depth_image_width;
	static const unsigned int depth_image_height;
	static const unsigned int num_channels_per_feed;
	static const unsigned int depth_channel_offset;
	static const unsigned int rgb_channel_offset;
	static const unsigned int rgb_element_size;
	static const unsigned int depth_element_size;
	static const unsigned int rgb_image_size;
	static const unsigned int depth_image_size;
	virtual ~kinect_v2_info();
};

} /* namespace workbench */
} /* namespace reco */

#endif /* APPLICATIONS_WORKBENCH_SRC_KINECT_V2_INFO_H_ */
