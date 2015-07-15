/*
 * multi_kinect_rgb_viewer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_DATAPIPE_MULTI_KINECT_RGB_VIEWER_H_
#define RECO_DATAPIPE_MULTI_KINECT_RGB_VIEWER_H_
#pragma once

#include <reco/datapipe/offset_channel_viewer.h>

namespace reco {
namespace datapipe {

class multi_kinect_rgb_viewer: public offset_channel_viewer<kinect_v2_info::channels.size(),
		kinect_v2_info::channel_type::RGB> {
Q_OBJECT

public:
	multi_kinect_rgb_viewer(QString window_title = "Feed Viewer", QWidget* parent = NULL);
	virtual ~multi_kinect_rgb_viewer();
};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_MULTI_KINECT_RGB_VIEWER_H_ */
