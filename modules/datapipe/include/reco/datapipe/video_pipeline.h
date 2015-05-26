/*
 * VideoPipeline.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef RECO_DATAPIPE_VIDEOPIPELINE_H_
#define RECO_DATAPIPE_VIDEOPIPELINE_H_

#pragma once

//Qt
#include <QObject>
//opencv
#include <opencv2/core/core.hpp>

//local
#include "video_source.h"
#include "base_video_pipeline.h"

//std
#include <memory>

namespace reco {
namespace datapipe {

template<class VS>
class video_pipeline:public base_video_pipeline {
	//TODO: why does this throws a "Macro argument mismatch" warning
	static_assert(std::is_base_of<video_source, VS>::value,
	        "VS must be a descendant of VideoSource"
	    );
private:

	bool stop_requested;

protected:
	std::shared_ptr<VS> current_video_source;
	/**
	 * Constructor.
	 * Memory management of videoSource is taken over by the pipeline, do not delete the videoSource after passing it in.
	 * @param videoSource - video source to retrieve frames from. See notes above about memory management of this object.
	 */
	video_pipeline(const std::shared_ptr<VS>& video_source_in):
		base_video_pipeline(),
		stop_requested(false),
		current_video_source(video_source_in){
	}
	virtual ~video_pipeline();

public slots:
	virtual void run();
	virtual void request_stop();

};

} /* namespace reco */
} /* namespace datapipe */

#include "video_pipeline.tpp"

#endif /* RECO_DATAPIPE_VIDEOPIPELINE_H_ */
