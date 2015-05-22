/*
 * VideoPipeline.tpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

#include "VideoPipeline.h"
//opencv
#include <opencv2/core/core.hpp>

namespace reco {
namespace vidgui {


template<class VS>
VideoPipeline<VS>::~VideoPipeline(){
	videoSource->tearDown();
	delete videoSource;
}

template<class VS>
void VideoPipeline<VS>::run() {
	using namespace cv;

	//reset stop flag
	stopRequested = false;
	if(!videoSource->setUp()){
		emit error("Could not connect to video source.");
	}

	while (!stopRequested) {
		//TODO: optimization target. Re-architecture to use a non-pointer videoSource member if you find it sensible.
		//Non-pointer members allow more use of inlining and sometimes result in fewer instructions.
		cv::Mat newFrame = videoSource->retrieveFrame();
		processFrame(newFrame);

		emit frameReady(newFrame);
	}
	videoSource->tearDown();
	emit finished();
}
template<class VS>
void VideoPipeline<VS>::requestStop() {
	stopRequested = true;
}

} /* namespace vidgui */
} /* namespace  reco*/
