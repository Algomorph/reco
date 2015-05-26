/*
 * VideoPipeline.tpp
 *
 *      Author: Gregory Kramida
 *     License: Apache v2
 *   Copyright: (c) Gregory Kramida 2015
 */

//local
#include <opencv2/core/core.hpp>
#include <reco/datapipe/video_pipeline.h>

namespace reco {
namespace datapipe {


template<class VS>
video_pipeline<VS>::~video_pipeline(){
}

template<class VS>
void video_pipeline<VS>::run() {
	using namespace cv;

	//reset stop flag
	stop_requested = false;

	if(!current_video_source->set_up()){
		emit error("Could not set up the video source.");
	}

	while (!stop_requested && current_video_source->capture_frame()) {
		cv::Mat new_frame = current_video_source->take_frame();
		process_frame(new_frame);
		emit frame_ready(new_frame);
	}
	current_video_source->tear_down();
	emit finished();
}
template<class VS>
void video_pipeline<VS>::request_stop() {
	stop_requested = true;
}

} /* namespace datapipe */
} /* namespace  reco*/
