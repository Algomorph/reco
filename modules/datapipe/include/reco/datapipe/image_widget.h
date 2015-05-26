/*
 * CvImageWidget.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_CVIMAGEWIDGET_H_
#define RECO_DATAPIPE_CVIMAGEWIDGET_H_
#pragma once

//qt
#include <QWidget>
#include <QImage>

//opencv
#include <opencv2/core/core.hpp>

namespace reco {
namespace datapipe {
class image_widget: public QWidget {
	Q_OBJECT
public:
	image_widget(QWidget *parent = 0);
	virtual ~image_widget();

	QSize sizeHint() const {
		return image.size();
	}
	QSize minimumSizeHint() const {
		return image.size();
	}

public slots:
	void setImage(const cv::Mat& image);
	void setImageAndResize(const cv::Mat& image);
	void setImageFast(const cv::Mat& image);

protected:
	void paintEvent(QPaintEvent* event);

	QImage image;
	cv::Mat tmp;

};
} /* namespace reco */
} /* namespace datapipe */

#endif /* RECO_DATAPIPE_CVIMAGEWIDGET_H_ */
