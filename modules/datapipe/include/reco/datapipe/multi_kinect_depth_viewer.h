/*
 * multi_kinect_depth_viewer.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_DATAPIPE_MULTI_KINECT_DEPTH_VIEWER_H_
#define RECO_DATAPIPE_MULTI_KINECT_DEPTH_VIEWER_H_
#pragma once

#include <reco/datapipe/offset_channel_viewer.h>
#include <reco/datapipe/kinect_v2_info.h>

namespace reco {
namespace datapipe {

class multi_kinect_depth_viewer: public offset_channel_viewer<kinect_v2_info::channels.size(),
		kinect_v2_info::channel_type::DEPTH> {
Q_OBJECT

public:
	multi_kinect_depth_viewer(QWidget* parent = NULL,QString window_title = "Kinect Depth Viewer");
	virtual ~multi_kinect_depth_viewer();

public slots:
	/**
	 * @brief triggered when a new frame becomes available.
	 * @param images
	 */
	virtual void on_frame(std::shared_ptr<hal::ImageArray> images);

};

} /* namespace datapipe */
} /* namespace reco */

#endif /* RECO_DATAPIPE_MULTI_KINECT_DEPTH_VIEWER_H_ */
