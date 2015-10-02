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
#ifdef USE_STEREO_SGBM
				stereo_matcher(0, 64, 3, 216, 864, -1, 48, 0, 0, 0, false),
#else
				stereo_matcher(cv::StereoBM::BASIC_PRESET,64,9),
#endif
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),
				worker_shutting_down(false),

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
		left.copyTo(last_left);
		right.copyTo(last_right);


#ifdef INHOUSE_RECTIFICATION

#ifdef USE_STEREO_SGBM
		cv::Mat rect_left(last_left.rows,last_left.cols,last_left.type());
		cv::Mat rect_right(last_right.rows,last_right.cols,last_right.type());
		calibu::Rectify(left_lut,last_left.data,rect_left.data, last_left.cols,last_left.rows, last_left.channels());
		calibu::Rectify(right_lut,last_right.data,rect_right.data, last_right.cols,last_right.rows, last_right.channels());

#else
		cv::cvtColor(last_left,last_left,CV_BGR2GRAY);
		cv::cvtColor(last_right,last_right,CV_BGR2GRAY);
		cv::Mat rect_left(left.rows,left.cols,left.type());
		cv::Mat rect_right(right.rows,right.cols,right.type());
		calibu::Rectify(left_lut,last_left.data,rect_left.data, last_left.cols,last_left.rows, 1);
		calibu::Rectify(right_lut,last_right.data,rect_right.data, last_right.cols,right.rows, 1);

#endif
		last_left = rect_left;
		last_right = rect_right;
#else

#endif
		compute_disparity(last_left,last_right);

		return true;
	}
	worker_shutting_down = true;
	return false;
}

/**
 * Compute disparity and emit ready frame
 * @param left
 * @param right
 */
void stereo_processor::compute_disparity(cv::Mat left, cv::Mat right){
	cv::Mat disparity;
	stereo_matcher(left, right, disparity);

	cv::Mat disparity_normalized;
	cv::Mat disparity_morphed;


	cv::normalize(disparity,disparity_normalized,0,255,cv::NORM_MINMAX, CV_8U);
	//stereo_matcher(rect_left, rect_right, disparity);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));
	cv::morphologyEx(disparity_normalized,disparity_morphed,cv::MORPH_CLOSE,kernel);
	cv::cvtColor(disparity_morphed,disparity_morphed,CV_GRAY2BGR);
	//cv::cvtColor(disparity_normalized,disparity_normalized,CV_GRAY2BGR);

	std::shared_ptr<std::vector<cv::Mat>> images(new std::vector<cv::Mat>());
	images->push_back(disparity_morphed);
	emit frame(images);
}

void stereo_processor::set_minimum_disparity(int value){
	stereo_matcher.minDisparity = value;
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_num_disparities(int value){
	stereo_matcher.numberOfDisparities = value - (value % 16);
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_window_size(int value){
	stereo_matcher.SADWindowSize = value + ((value + 1) % 2);
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_p1(int value){
	if(value < stereo_matcher.P2){
		stereo_matcher.P1 = value;
		compute_disparity(last_left,last_right);
	}
}
void stereo_processor::set_p2(int value){
	if(value > stereo_matcher.P1){
		stereo_matcher.P2 = value;
		compute_disparity(last_left,last_right);
	}
}
void stereo_processor::set_pre_filter_cap(int value){
	stereo_matcher.preFilterCap = value;
	compute_disparity(last_left,last_right);
}

void stereo_processor::set_uniqueness_ratio(int value){
	stereo_matcher.uniquenessRatio = value;
	compute_disparity(last_left,last_right);
}

void stereo_processor::set_speckle_window_size(int value){
	stereo_matcher.speckleRange = value;
	compute_disparity(last_left,last_right);
}

void stereo_processor::set_speckle_range(int value){
	stereo_matcher.speckleRange = value;
	compute_disparity(last_left,last_right);
}





} /* namespace stereo_workbench */
} /* namespace reco */
