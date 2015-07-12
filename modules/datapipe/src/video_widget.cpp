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
video_widget::video_widget(QWidget *parent) : image_widget(parent) {}

video_widget::~video_widget() {}

void video_widget::connect_to_video_pipeline(const base_video_pipeline* pipeline){
	connect(pipeline, SIGNAL(frameReady(cv::Mat)),this, SLOT(setImageFast(cv::Mat)));
}

} /* namespace datapipe */
} /* namespace reco */
