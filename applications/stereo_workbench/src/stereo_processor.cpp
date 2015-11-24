/*
 * stereo_processor.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//utils
#include <reco/utils/queue.h>
#include <reco/utils/debug_util.h>
#include <reco/utils/cpp_exception_util.h>
//std
#include <memory>

//calibu
#include <calibu/Calibu.h>
#include <calibu/cam/stereo_rectify.h>
//sophus
#include <sophus/se3.hpp>
//eigen
#include <Eigen/Eigen>
//opencv
#include <opencv2/highgui/highgui.hpp>
#include <reco/stereo_workbench/stereo_processor.hpp>


namespace reco {
namespace stereo_workbench {

stereo_processor::stereo_processor(
		datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<matcher_qt_wrapper_base> matcher,
		std::shared_ptr<stereo::rectifier> rectifier) :
				worker(),
				matcher(matcher),
				rectification_enabled((bool) rectifier),
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),

				worker_shutting_down(false),
				right_v_offset(0),
				_rectifier(rectifier)
{
	if(matcher){
		connect(matcher.get(), SIGNAL(parameters_changed()), this, SLOT(recompute_disparity()));
	}
}

stereo_processor::~stereo_processor() {

}

void stereo_processor::set_rectifier(std::shared_ptr<stereo::rectifier> _rectifier) {
	std::unique_lock<std::mutex> lck(this->input_guard);
	this->_rectifier = _rectifier;
	if (!_rectifier) {
		rectification_enabled = false;
	}
	if (!last_left.empty()) {
		if (rectification_enabled) {
			_rectifier->rectify(last_left, last_right, last_left_rectified, last_right_rectified);
			compute_disparity(last_left_rectified, last_right_rectified);
		} else {
			compute_disparity(last_left, last_right);
		}
	}
}

void stereo_processor::set_matcher(std::shared_ptr<matcher_qt_wrapper_base> matcher) {

	this->matcher = matcher;
	if (!matcher) {
		err2(std::invalid_argument,"matcher contents cannot be null");
	}
	connect(matcher.get(), SIGNAL(parameters_changed()), this, SLOT(recompute_disparity()));
	emit matcher_updated(matcher.get());
	recompute_disparity();
}

void stereo_processor::pre_thread_join() {

	std::shared_ptr<hal::ImageArray> dummy;
	//run through a "fake" frame to queue to ensure thread doesn't get
	//stuck on popping from an empty queue
	input_frame_buffer->clear();
	input_frame_buffer->push_back(dummy);
}

std::shared_ptr<matcher_qt_wrapper_base> stereo_processor::get_matcher() const {
	return matcher;
}

bool stereo_processor::do_unit_of_work() {
	std::shared_ptr<hal::ImageArray> array = input_frame_buffer->pop_front();
	if (array) {
		//for now, assume stereo pair
		cv::Mat left = *(array->at(0));
		cv::Mat right = *(array->at(1));

		std::unique_lock<std::mutex> lck(this->input_guard);
		//TODO try without copying
		left.copyTo(last_left);
		right.copyTo(last_right);

		if (rectification_enabled) {
			//protect _rectifier from write access during rectification
			_rectifier->rectify(last_left, last_right, last_left_rectified, last_right_rectified);
			compute_disparity(last_left_rectified, last_right_rectified);
		} else {
			compute_disparity(last_left, last_right);
		}

		return true;
	}
	worker_shutting_down = true;
	return false;
}

void stereo_processor::save_current_matcher_input() {
	std::unique_lock<std::mutex> lck(this->input_guard);
	if (rectification_enabled ) {
		cv::imwrite("left.png", last_left_rectified);
		cv::imwrite("right.png", last_right_rectified);
	} else {
		cv::imwrite("left.png", last_left);
		cv::imwrite("right.png", last_right);
	}
	if (!disparity_normalized.empty()) {
		cv::imwrite("output.png", disparity_normalized);
	}
}

void stereo_processor::recompute_disparity() {
	std::unique_lock<std::mutex> lck(this->input_guard);
	if(!last_left.empty()){
		if (rectification_enabled) {
			compute_disparity(last_left_rectified, last_right_rectified);
		} else {
			compute_disparity(last_left, last_right);
		}
	}
}

bool stereo_processor::is_rectification_enabled() const {
	return this->rectification_enabled;
}

/**
 * Compute disparity and emit ready frame
 * @param left
 * @param right
 */
void stereo_processor::compute_disparity(cv::Mat left, cv::Mat right) {
	if(!matcher){
		return;
	}
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

	matcher->compute(left, right, disparity);

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

int stereo_processor::get_v_offset() const {
	return this->right_v_offset;
}

void stereo_processor::set_v_offset(int value) {
	right_v_offset = value;
	recompute_disparity();
}

void stereo_processor::toggle_rectification() {
	if (this->_rectifier) {
		rectification_enabled = !rectification_enabled;
		std::unique_lock<std::mutex> lck(this->input_guard);
		if (!last_left.empty()) {
			if (rectification_enabled) {
				_rectifier->rectify(last_left, last_right, last_left_rectified, last_right_rectified);
				compute_disparity(last_left_rectified, last_right_rectified);
			} else {
				compute_disparity(last_left, last_right);
			}
		}
	}
}

} /* namespace stereo_workbench */
} /* namespace reco */
