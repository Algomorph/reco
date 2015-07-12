/*
 * CvVideoWidget.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_CVVIDEOWIDGET_H_
#define RECO_DATAPIPE_CVVIDEOWIDGET_H_

#pragma once
//qt
#include <QWidget>
//local
#include <reco/datapipe/image_widget.h>
#include <reco/datapipe/video_pipeline.h>

namespace reco {
namespace datapipe {

class video_widget: public image_widget {
	Q_OBJECT
public:
	video_widget(QWidget *parent = NULL);
	virtual ~video_widget();
	void connect_to_video_pipeline(const base_video_pipeline* pipeline);

};
} /* namespace video */
} /* namespace augmentarium */

#endif /* RECO_DATAPIPE_CVVIDEOWIDGET_H_ */
