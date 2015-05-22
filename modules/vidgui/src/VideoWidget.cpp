/*
 * CvVideoWidget.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#include <reco/vidgui/VideoWidget.h>
#include <QDebug>

namespace reco {
namespace vidgui{
VideoWidget::VideoWidget(QWidget *parent) : ImageWidget(parent) {
	// TODO Auto-generated constructor stub

}

VideoWidget::~VideoWidget() {
	// TODO Auto-generated destructor stub
}

void VideoWidget::connectToVideoPipeline(const BaseVideoPipeline* pipeline){
	connect(pipeline, SIGNAL(frameReady(cv::Mat)),this, SLOT(setImageFast(cv::Mat)));
}

} /* namespace vidgui */
} /* namespace reco */
