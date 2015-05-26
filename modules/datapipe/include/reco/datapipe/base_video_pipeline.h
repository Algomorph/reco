#ifndef RECO_DATAPIPE_BASEVIDEOPIPELINE_H_
#define RECO_DATAPIPE_BASEVIDEOPIPELINE_H_

#pragma once

#include <opencv2/core/core.hpp>
#include <QObject>

namespace reco {
namespace datapipe {

class base_video_pipeline: public QObject{
	Q_OBJECT
protected:
	base_video_pipeline(){
	};
	virtual ~base_video_pipeline(){};
	virtual void process_frame(const cv::Mat& frame) = 0;
public slots:
	virtual void run() = 0;
	virtual void request_stop() = 0;
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
	void frame_ready(cv::Mat);
	/**
	 * Emitted when some kind of new result image was prepared
	 * @param
	 */
	void result_image_ready(cv::Mat);
	void finished();
};

}/*namespace video*/
}/*namespace augmentarium*/
#endif
