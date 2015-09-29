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
//calibu
#include <calibu/Calibu.h>
#include <calibu/cam/stereo_rectify.h>
//sophus
#include <sophus/se3.hpp>

namespace reco {
namespace stereo_workbench {

stereo_processor::stereo_processor(
		datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<calibu::Rigd> calibration)
:
		worker(),
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),
				worker_shutting_down(false),
#ifdef USE_STEREO_SGBM
				stereo_matcher(0, 64, 8, 1536, 6144, 0, 0, 0, 0, 0, false),
#else
				stereo_matcher(cv::StereoBM::BASIC_PRESET,64,9),
#endif
				calibration(calibration)
//stereo_matcher(32, 8)
{
	Sophus::SE3d T_nr_nl;

	calibu::CreateScanlineRectifiedLookupAndCameras(
			calibration->cameras_[1]->Pose(),
			calibration->cameras_[0],
			calibration->cameras_[1],
			T_nr_nl, left_lut, right_lut
			);
}

stereo_processor::~stereo_processor() {

}
void stereo_processor::pre_thread_join() {

	std::shared_ptr<hal::ImageArray> dummy;
	//run through a "fake" frame to queue to ensure thread doesn't get
	//stuck on popping from an empty queue
	input_frame_buffer->clear();
	input_frame_buffer->push_back(dummy);
}

bool stereo_processor::do_unit_of_work() {
	std::shared_ptr<hal::ImageArray> array = input_frame_buffer->pop_front();
	if (array) {
		//for now, assume stereo pair
		cv::Mat left = *(array->at(0));
		cv::Mat right = *(array->at(1));

		cv::Mat copy_left, copy_right;
		left.copyTo(copy_left);
		right.copyTo(copy_right);


#ifdef USE_STEREO_SGBM
		cv::Mat rect_left(left.rows,left.cols,left.type());
		cv::Mat rect_right(right.rows,right.cols,right.type());
		calibu::Rectify(left_lut,copy_left.data,rect_left.data, left.cols,left.rows, left.channels());
		calibu::Rectify(right_lut,copy_right.data,rect_right.data, right.cols,right.rows, right.channels());

#else
		cv::cvtColor(left,left,CV_BGR2GRAY);
		cv::cvtColor(right,right,CV_BGR2GRAY);
		cv::Mat rect_left(left.rows,left.cols,left.type());
		cv::Mat rect_right(right.rows,right.cols,right.type());
		calibu::Rectify(left_lut,left.data,rect_left.data, left.cols,left.rows, 1);
		calibu::Rectify(right_lut,right.data,rect_right.data, right.cols,right.rows, 1);
#endif


		cv::Mat disparity;
		stereo_matcher(rect_left, rect_right, disparity);
		double min, max;
		cv::minMaxLoc(disparity, &min, &max);
		//puts("min " << min << std::endl << "max " << max);
		cv::Mat res;
		disparity += 16;
		cv::convertScaleAbs(disparity, res, 0.25);
		cv::cvtColor(res, res, CV_GRAY2BGR);
		std::shared_ptr<std::vector<cv::Mat>> images(new std::vector<cv::Mat>());
		images->push_back(res);
		emit frame(images);
		return true;
	}
	worker_shutting_down = true;
	return false;
}

} /* namespace stereo_workbench */
} /* namespace reco */
