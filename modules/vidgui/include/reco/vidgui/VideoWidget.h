/*
 * CvVideoWidget.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_CVVIDEOWIDGET_H_
#define MODULES_VIDEO_CVVIDEOWIDGET_H_
#pragma once

#include "ImageWidget.h"
#include "VideoPipeline.h"

namespace reco {
namespace vidgui {

class VideoWidget: public ImageWidget {
	Q_OBJECT
public:
	VideoWidget(QWidget *parent = NULL);
	virtual ~VideoWidget();
	void connectToVideoPipeline(const BaseVideoPipeline* pipeline);

};
} /* namespace video */
} /* namespace augmentarium */

#endif /* MODULES_VIDEO_CVVIDEOWIDGET_H_ */
