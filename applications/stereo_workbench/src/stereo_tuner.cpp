/*
 * stereo_processor.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <reco/utils/queue.h>
//std
#include <memory>
#include <reco/utils/debug_util.h>
//calibu
#include <calibu/Calibu.h>
#include <calibu/cam/stereo_rectify.h>
//sophus
#include <sophus/se3.hpp>
//eigen
#include <Eigen/Eigen>
//opencv
#include <opencv2/highgui/highgui.hpp>
#include "stereo_tuner.h"

namespace reco {
namespace stereo_workbench {

stereo_tuner::stereo_tuner(
		datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<rectifier>  rectifier)
:
		worker(),

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)

				stereo_matcher(0, 64, 3, 216, 864, -1, 48, 0, 0, 0, false),
#elif CV_VERSION_MAJOR == 3
				stereo_matcher(
						cv::StereoSGBM::create(0, 64, 3, 216, 864, -1, 48, 0, 0, 0,
								cv::StereoSGBM::MODE_SGBM)),

#endif
				rectification_enabled((bool)rectifier),
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),
				worker_shutting_down(false),
				right_v_offset(0),

				_rectifier(rectifier)

{
}

stereo_tuner::~stereo_tuner() {

}

void stereo_tuner::set_rectifier(std::shared_ptr<rectifier> _rectifier) {
	std::unique_lock<std::mutex> lckr(this->rectify_guard);
	this->_rectifier = _rectifier;
	if(!_rectifier){
		rectification_enabled = false;
	}
	std::unique_lock<std::mutex> lckp(this->pause_mutex);
	if(!last_left.empty() && this->is_paused()){
		if(rectification_enabled){
			_rectifier->rectify(last_left,last_right,last_left_rectified,last_right_rectified);
		}
		recompute_disparity();
	}
}

void stereo_tuner::pre_thread_join() {

	std::shared_ptr<hal::ImageArray> dummy;
	//run through a "fake" frame to queue to ensure thread doesn't get
	//stuck on popping from an empty queue
	input_frame_buffer->clear();
	input_frame_buffer->push_back(dummy);
}

bool stereo_tuner::do_unit_of_work() {
	std::shared_ptr<hal::ImageArray> array = input_frame_buffer->pop_front();
	if (array) {
		//for now, assume stereo pair
		cv::Mat left = *(array->at(0));
		cv::Mat right = *(array->at(1));


		left.copyTo(last_left);
		right.copyTo(last_right);

		{
			//protect _rectifier from write access during rectification
			std::unique_lock<std::mutex> lck(this->rectify_guard);
			if (rectification_enabled) {
				_rectifier->rectify(last_left,last_right,last_left_rectified,last_right_rectified);
			}
		}
		recompute_disparity();

		return true;
	}
	worker_shutting_down = true;
	return false;
}

void stereo_tuner::save_current() {
	if(rectification_enabled){
		cv::imwrite("left.png", last_left_rectified);
		cv::imwrite("right.png", last_right_rectified);
	}else{
		cv::imwrite("left.png", last_left);
		cv::imwrite("right.png", last_right);
	}
}


void stereo_tuner::recompute_disparity(){
	if(rectification_enabled){
		compute_disparity(last_left_rectified,last_right_rectified);
	}else{
		compute_disparity(last_left,last_right);
	}
}

void stereo_tuner::recompute_disparity_if_paused(){
	//protect the paused variable from being written during recompute

	std::unique_lock<std::mutex> lck(this->pause_mutex);
	if(this->is_paused()){
		recompute_disparity();
	}
}

/**
 * Compute disparity and emit ready frame
 * @param left
 * @param right
 */
void stereo_tuner::compute_disparity(cv::Mat left, cv::Mat right) {
	cv::Mat disparity;

	int offset = this->right_v_offset;
	if (right_v_offset != 0) {
		cv::Mat right_adjusted = cv::Mat(right.rows, right.cols, right.type(), cv::Scalar(0, 0, 0));
		if (offset > 0) {
			right.rowRange(cv::Range(offset, right.rows))
					.copyTo(right_adjusted.rowRange(cv::Range(0, right.rows - offset)));
		} else {
			right.rowRange(cv::Range(0, right.rows + offset))
					.copyTo(right_adjusted.rowRange(cv::Range(-offset, right.rows)));
		}
		right = right_adjusted;
	}

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)
	stereo_matcher(left, right, disparity);
#elif CV_VERSION_MAJOR == 3
	stereo_matcher->compute(left, right, disparity);
#endif

	cv::Mat disparity_normalized;

	cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
#ifdef MORPHOLOGY_CLOSE
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));
	cv::morphologyEx(disparity_normalized,disparity_normalized,cv::MORPH_CLOSE,kernel);
#endif
	cv::cvtColor(disparity_normalized, disparity_normalized, CV_GRAY2BGR);

	std::shared_ptr<std::vector<cv::Mat>> images(new std::vector<cv::Mat>());
	images->push_back(disparity_normalized);
	emit frame(images);
}

#if CV_VERSION_MAJOR == 3
void stereo_tuner::compute_disparity_daisy(cv::Mat left, cv::Mat right) {

}
#endif

int stereo_tuner::get_v_offset() {
	return this->right_v_offset;
}

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)
void stereo_tuner::set_minimum_disparity(int value) {
	stereo_matcher.minDisparity = value;
	recompute_disparity_if_paused();
}
void stereo_tuner::set_num_disparities(int value) {
	stereo_matcher.numberOfDisparities = value - (value % 16);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_window_size(int value) {
	stereo_matcher.SADWindowSize = value + ((value + 1) % 2);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_p1(int value) {
	if(value < stereo_matcher.P2) {
		stereo_matcher.P1 = value;
		recompute_disparity_if_paused();
	}
}
void stereo_tuner::set_p2(int value) {
	if(value > stereo_matcher.P1) {
		stereo_matcher.P2 = value;
		recompute_disparity_if_paused();
	}
}
void stereo_tuner::set_pre_filter_cap(int value) {
	stereo_matcher.preFilterCap = value;
	recompute_disparity_if_paused();
}
void stereo_tuner::set_uniqueness_ratio(int value) {
	stereo_matcher.uniquenessRatio = value;
	recompute_disparity_if_paused();
}
void stereo_tuner::set_speckle_window_size(int value) {
	stereo_matcher.speckleRange = value;
	recompute_disparity_if_paused();
}
void stereo_tuner::set_speckle_range(int value) {
	stereo_matcher.speckleRange = value;
	recompute_disparity_if_paused();
}
#elif CV_VERSION_MAJOR == 3
void stereo_tuner::set_minimum_disparity(int value) {
	stereo_matcher->setMinDisparity(value);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_num_disparities(int value) {
	stereo_matcher->setNumDisparities(value - (value % 16));
	recompute_disparity_if_paused();
}
void stereo_tuner::set_window_size(int value) {
	int new_window_size = value + ((value + 1) % 2);
	stereo_matcher->setBlockSize(new_window_size);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_p1(int value) {
	if (value < stereo_matcher->getP2()) {
		stereo_matcher->setP1(value);
		recompute_disparity_if_paused();
	}
}
void stereo_tuner::set_p2(int value) {
	if (value > stereo_matcher->getP1()) {
		stereo_matcher->setP2(value);
		recompute_disparity_if_paused();
	}
}
void stereo_tuner::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_speckle_window_size(int value) {
	stereo_matcher->setSpeckleWindowSize(value);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_speckle_range(int value) {
	stereo_matcher->setSpeckleRange(value);
	recompute_disparity_if_paused();
}
void stereo_tuner::set_v_offset(int value) {
	right_v_offset = value;
	recompute_disparity_if_paused();
}
void stereo_tuner::toggle_rectification(){
	if(this->_rectifier){
		rectification_enabled = !rectification_enabled;
		std::unique_lock<std::mutex> lck(pause_mutex);
		if(this->is_paused()){
			if(rectification_enabled){
				_rectifier->rectify(last_left,last_right,last_left_rectified,last_right_rectified);
			}
			recompute_disparity();
		}
	}
}

#endif

} /* namespace stereo_workbench */
} /* namespace reco */
