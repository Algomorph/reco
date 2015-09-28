/*
 * stereo_processor.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <src/stereo_processor.h>
//utils
#include <reco/utils/queue.h>
//std
#include <memory>
#include <reco/utils/debug_util.h>

namespace reco {
namespace stereo_workbench {

stereo_processor::stereo_processor(datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer)
:
		worker(),
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),
				stereo_matcher(0, 64, 8, 1536, 6144, 0, 0, 0, 0, 0, false)
				//stereo_matcher(32, 8)
	{
}

stereo_processor::~stereo_processor() {

}
void stereo_processor::pre_thread_join() {
	//run through a "fake" frame to queue to ensure thread doesn't get
	//stuck on popping from an empty queue
	std::shared_ptr<hal::ImageArray> dummy;
	input_frame_buffer->push_back(dummy);
}

bool stereo_processor::do_unit_of_work() {
	std::shared_ptr<hal::ImageArray> array = input_frame_buffer->pop_front();
	if (array) {
		//for now, assume stereo pair
		cv::Mat left = *(array->at(0));
		cv::Mat right = *(array->at(1));
		//cv::Mat copyLeft;
		//cv::Mat copyRight;
		//left.copyTo(copyLeft);
		//right.copyTo(copyRight);
		cv::Mat disparity;
		//stereo_matcher(copyLeft, copyRight, disparity);
		stereo_matcher(left, right, disparity);
		double min, max;
		cv::minMaxLoc(disparity,&min,&max);
		puts("min " << min << std::endl << "max " << max);
		cv::Mat res;
		disparity+=16;
		cv::convertScaleAbs(disparity,res, 0.25);
		cv::cvtColor(res,res,CV_GRAY2BGR);
		std::shared_ptr<std::vector<cv::Mat>> images(new std::vector<cv::Mat>());
		images->push_back(res);
		emit frame(images);
		return true;
	}
	return false;
}

} /* namespace stereo_workbench */
} /* namespace reco */
