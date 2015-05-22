#ifndef MODULES_VIDEO_BASEVIDEOPIPELINE_H_
#define MODULES_VIDEO_BASEVIDEOPIPELINE_H_

#pragma once

#include <opencv2/core/core.hpp>
#include <QObject>

namespace reco {
namespace vidgui {

class BaseVideoPipeline: public QObject{
	Q_OBJECT
protected:
	BaseVideoPipeline(){
	};
	virtual ~BaseVideoPipeline(){};
	virtual void processFrame(const cv::Mat& frame) = 0;
public slots:
	virtual void run() = 0;
	virtual void requestStop() = 0;
signals:
/**
 * Emitted on error
 * @param error
 */
	void error(QString err);
	/**
	 * Emitted when a new frame had been processed
	 * @param
	 */
	void frameReady(cv::Mat);
	/**
	 * Emitted when some kind of new result image was prepared
	 * @param
	 */
	void resultImageReady(cv::Mat);
	void finished();
};

}/*namespace video*/
}/*namespace augmentarium*/
#endif
