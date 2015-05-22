/*
 * CvImageWidget.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_CVIMAGEWIDGET_H_
#define MODULES_VIDEO_CVIMAGEWIDGET_H_
#pragma once

#include <QWidget>
#include <QImage>
#include <opencv2/core/core.hpp>

namespace reco {
namespace vidgui {
class ImageWidget: public QWidget {
	Q_OBJECT
public:
	ImageWidget(QWidget *parent = 0);
	virtual ~ImageWidget();

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
} /* namespace video */
} /* namespace augmentarium */

#endif /* MODULES_VIDEO_CVIMAGEWIDGET_H_ */
