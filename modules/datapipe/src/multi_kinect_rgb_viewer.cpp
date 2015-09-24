/*
 * multi_kinect_rgbviewer.cpp
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#include <reco/datapipe/multi_kinect_rgb_viewer.h>

namespace reco {
namespace datapipe {

multi_kinect_rgb_viewer::multi_kinect_rgb_viewer(QWidget* parent, QString window_title) :
				offset_channel_viewer<kinect_v2_info::channels.size(),
						kinect_v2_info::channel_type::RGB>(parent, window_title) {
}

multi_kinect_rgb_viewer::~multi_kinect_rgb_viewer() {
}

} /* namespace datapipe */
} /* namespace reco */
