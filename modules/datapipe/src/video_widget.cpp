/*
 * CvVideoWidget.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#include <reco/datapipe/video_widget.h>
#include <QDebug>

namespace reco {
namespace datapipe{
video_widget::video_widget(QWidget *parent) : image_widget(parent) {
	// TODO Auto-generated constructor stub

}

//VideoWidget::~VideoWidget() {
	// TODO Auto-generated destructor stub
//}

void video_widget::connectToVideoPipeline(const base_video_pipeline* pipeline){
	connect(pipeline, SIGNAL(frameReady(cv::Mat)),this, SLOT(setImageFast(cv::Mat)));
}

} /* namespace datapipe */
} /* namespace reco */
