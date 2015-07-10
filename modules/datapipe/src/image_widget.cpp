/*
 * CvImageWidget.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <reco/datapipe/image_widget.h>
#include <reco/utils/cpp_exception_util.h>

//qt
#include <QPainter>
#include <QPaintEvent>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

//std
//#ifdef _DEBUG
#include <iostream>
//#endif

namespace reco {
namespace datapipe {
image_widget::image_widget(QWidget *parent) :
				QWidget(parent) {
}

image_widget::~image_widget() {
	// TODO Auto-generated destructor stub
}

#define CV_IMAGE_WIDGET_CONF_MAT(mat) 	\
	switch (mat.type()) {				\
	case CV_8UC1:						\
		cvtColor(mat, tmp, CV_GRAY2RGB);\
		break;							\
	case CV_8UC3:						\
		cvtColor(mat, tmp, CV_BGR2RGB);	\
		break;							\
	}

/**
 * Fast version of setImage: assumes BGR mat (CV_8UC3), does not resize widget
 * @param mat matrix to use for current image
 */
void image_widget::set_image_fast(const cv::Mat& mat) {

	// Convert the image to the RGB888 format
	// assume BGR image
	if(mat.type() != CV_8UC3){
		err(std::invalid_argument) << "Error caught in image_widget::set_image_fast...Wrong image type: " << mat.type() << "." << std::endl << enderr;
	}

	cvtColor(mat, tmp, CV_BGR2RGB);

	// Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
	// is 3*width because each pixel has three bytes.
	image = QImage(tmp.data, tmp.cols, tmp.rows, tmp.cols * 3, QImage::Format_RGB888);

	repaint();
}

void image_widget::set_blank(uint width, uint height){

	image = QImage(QSize(width,height), QImage::Format_RGB888);
	repaint();
}

/**
 * Sets the current image
 * @param mat
 */
void image_widget::set_image(const cv::Mat& mat) {

	// Convert the image to the RGB888 format
	CV_IMAGE_WIDGET_CONF_MAT(mat)

	// QImage needs the data to be stored continuously in memory
	assert(tmp.isContinuous());

	// Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
	// is 3*width because each pixel has three bytes.
	image = QImage(tmp.data, tmp.cols, tmp.rows, tmp.cols * 3, QImage::Format_RGB888);

	repaint();
}


/**
 * Sets the current image and resizes the widget to the size of this image
 * @param mat
 */
void image_widget::set_image_and_resize(const cv::Mat& mat) {

	// Convert the image to the RGB888 format
	CV_IMAGE_WIDGET_CONF_MAT(mat)

	// QImage needs the data to be stored continuously in memory
	assert(tmp.isContinuous());
	// Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
	// is 3*width because each pixel has three bytes.
	image = QImage(tmp.data, tmp.cols, tmp.rows, tmp.cols * 3, QImage::Format_RGB888);

	this->setFixedSize(mat.cols, mat.rows);

	repaint();
}
void image_widget::paintEvent(QPaintEvent* event) {
	QWidget::paintEvent(event);
	// Display the image
	QPainter painter(this);

	QSize widgetSize = event->rect().size();

	if(!image.isNull()){//to avoid annoying "QImage::scaled: Image is a null image " warnings
		painter.drawImage(QPoint(0, 0), image.scaled(widgetSize, Qt::KeepAspectRatio));
	}

	painter.end();
}
} /* namespace datapipe */
} /* namespace reco */

