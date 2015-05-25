/*
 * CvVideoWidget.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#include <reco/datapipe/VideoWidget.h>
#include <QDebug>

namespace reco {
namespace datapipe{
VideoWidget::VideoWidget(QWidget *parent) : ImageWidget(parent) {
	// TODO Auto-generated constructor stub

}

VideoWidget::~VideoWidget() {
	// TODO Auto-generated destructor stub
}

void VideoWidget::connectToVideoPipeline(const BaseVideoPipeline* pipeline){
	connect(pipeline, SIGNAL(frameReady(cv::Mat)),this, SLOT(setImageFast(cv::Mat)));
}

} /* namespace datapipe */
} /* namespace reco */
