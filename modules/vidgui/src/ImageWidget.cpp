/*
 * CvImageWidget.cpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <reco/vidgui/ImageWidget.h>

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
namespace vidgui {
ImageWidget::ImageWidget(QWidget *parent) :
				QWidget(parent) {
}

ImageWidget::~ImageWidget() {
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
void ImageWidget::setImageFast(const cv::Mat& mat) {

	// Convert the image to the RGB888 format
	// assume BGR image
	if(mat.type() != CV_8UC3){
		std::cout <<  "Image type: " << mat.type() << "." << std::endl;
	}

	cvtColor(mat, tmp, CV_BGR2RGB);

	// Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
	// is 3*width because each pixel has three bytes.
	image = QImage(tmp.data, tmp.cols, tmp.rows, tmp.cols * 3, QImage::Format_RGB888);

	repaint();
}

/**
 * Sets the current image
 * @param mat
 */
void ImageWidget::setImage(const cv::Mat& mat) {

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
void ImageWidget::setImageAndResize(const cv::Mat& mat) {

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
void ImageWidget::paintEvent(QPaintEvent* event) {
	QWidget::paintEvent(event);
	// Display the image
	QPainter painter(this);

	QSize widgetSize = event->rect().size();

	if(!image.isNull()){//to avoid annoying "QImage::scaled: Image is a null image " warnings
		painter.drawImage(QPoint(0, 0), image.scaled(widgetSize, Qt::KeepAspectRatio));
	}

	painter.end();
}
} /* namespace vidgui */
} /* namespace reco */

