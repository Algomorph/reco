/*
 * kinect_v2_info.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#include <reco/workbench/kinect_v2_info.h>

namespace reco {
namespace workbench {

// kinect v2 resolution
const unsigned int kinect_v2_info::depth_image_width = 512;
const unsigned int kinect_v2_info::depth_image_height = 424;
const unsigned int kinect_v2_info::rgb_image_width = 1920;
const unsigned int kinect_v2_info::rgb_image_height = 1080;
// assumed channel offsets
const unsigned int kinect_v2_info::num_channels_per_feed = 2;
const unsigned int kinect_v2_info::depth_channel_offset = 1;
const unsigned int kinect_v2_info::rgb_channel_offset = 0;

const unsigned int kinect_v2_info::rgb_element_size = 3;
const unsigned int kinect_v2_info::depth_element_size = 4;

const unsigned int kinect_v2_info::rgb_image_size = kinect_v2_info::rgb_image_width * kinect_v2_info::rgb_image_height * kinect_v2_info::rgb_element_size;
const unsigned int kinect_v2_info::depth_image_size = kinect_v2_info::depth_image_width * kinect_v2_info::depth_image_height * kinect_v2_info::depth_element_size;


//inaccessible
kinect_v2_info::kinect_v2_info()
{}

kinect_v2_info::~kinect_v2_info()
{}

} /* namespace workbench */
} /* namespace reco */
