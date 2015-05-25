/*
 * VideoPipeline.h
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#ifndef MODULES_VIDEO_VIDEOPIPELINE_H_
#define MODULES_VIDEO_VIDEOPIPELINE_H_

#pragma once

//Qt
#include <QObject>
//opencv
#include <opencv2/core/core.hpp>

//local
#include "VideoSource.h"
#include "BaseVideoPipeline.h"


namespace reco {
namespace datapipe {

template<class VS>
class VideoPipeline:public BaseVideoPipeline {
	//TODO: why does this throws a "Macro argument mismatch" warning
	static_assert(std::is_base_of<VideoSource, VS>::value,
	        "VS must be a descendant of VideoSource"
	    );
private:


	bool stopRequested;

protected:
	VS* videoSource;
	//TODO: fix design problems with videoSource, change videoSource to non-pointer member by somehow copying it or passing initialization paramers to it.
	/**
	 * Constructor.
	 * Memory management of videoSource is taken over by the pipeline, do not delete the videoSource after passing it in.
	 * Do not use the same videoSource for two separate VideoPipeline objects, as that will cause double deallocation.
	 * @param videoSource - video source to retrieve frames from. See notes above about memory management of this object.
	 */
	VideoPipeline(VS* videoSource):
		BaseVideoPipeline(),
		stopRequested(false),
		videoSource(videoSource)
		{
	}
	virtual ~VideoPipeline();

public slots:
	virtual void run();
	virtual void requestStop();

};

} /* namespace video */
} /* namespace augmentarium */

#include "VideoPipeline.tpp"

#endif /* MODULES_VIDEO_VIDEOPIPELINE_H_ */
