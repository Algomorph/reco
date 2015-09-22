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

const data_channel kinect_v2_info::rgb_channel("RGB channel",1920,1080,RGB,3);
const data_channel kinect_v2_info::depth_channel("depth channel",512,424,DEPTH,4);
const std::array<const data_channel*,2> kinect_v2_info::channels({&rgb_channel,&depth_channel});
const float kinect_v2_info::depth_inv_factor = 4500.0f / 255.0f;

//inaccessible
kinect_v2_info::kinect_v2_info()
{}

kinect_v2_info::~kinect_v2_info()
{}

} /* namespace datapipe */
} /* namespace reco */
