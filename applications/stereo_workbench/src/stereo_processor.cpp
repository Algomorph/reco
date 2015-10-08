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
//eigen
#include <Eigen/Eigen>
//opencv
#include <opencv2/highgui/highgui.hpp>

namespace reco {
namespace stereo_workbench {

stereo_processor::stereo_processor(
		datapipe::frame_buffer_type input_frame_buffer,
		datapipe::frame_buffer_type output_frame_buffer,
		std::shared_ptr<calibu::Rigd> calibration = std::shared_ptr<calibu::Rigd>())
:
		worker(),

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)

				stereo_matcher(0, 64, 3, 216, 864, -1, 48, 0, 0, 0, false),
#elif CV_VERSION_MAJOR == 3
				stereo_matcher(
						cv::StereoSGBM::create(0, 64, 3, 216, 864, -1, 48, 0, 0, 0,
								cv::StereoSGBM::MODE_SGBM)),

#endif
				rectification_enabled(true),
				input_frame_buffer(input_frame_buffer),
				output_frame_buffer(output_frame_buffer),
				worker_shutting_down(false),
				right_v_offset(0),

				calibration(calibration)
//stereo_matcher(32, 8)
{
	if (calibration) {
		set_calibration(calibration);
	}
}

stereo_processor::~stereo_processor() {

}

void stereo_processor::set_calibration(std::shared_ptr<calibu::Rigd> calibration) {
	Sophus::SE3d T_nr_nl;
	calibu::CreateScanlineRectifiedLookupAndCameras(
			calibration->cameras_[1]->Pose(),
			calibration->cameras_[0],
			calibration->cameras_[1],
			//T_nr_nl, left_lut, right_lut
			T_nr_nl, right_lut, left_lut
			);
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

		cv::Mat copy_left(left.rows, left.cols, left.type());
		cv::Mat copy_right(right.rows, right.cols, right.type());
		left.copyTo(copy_left);
		right.copyTo(copy_right);

		if (rectification_enabled) {
			bool sizes_match = left_lut.Width() != copy_left.cols
					|| left_lut.Height() != copy_left.rows
					|| right_lut.Width() != copy_right.cols
					|| right_lut.Height() != copy_right.rows;

			if (!sizes_match) {
				puts("Lookup table sizes don't match image sizes. Skipping rectification.");
				last_left = copy_left;
				last_right = copy_right;
			} else {
				cv::Mat rect_left(left.rows, left.cols, left.type());
				cv::Mat rect_right(right.rows, right.cols, right.type());
				calibu::Rectify(left_lut, copy_left.data, rect_left.data, copy_left.cols,
						copy_left.rows, copy_left.channels());
				calibu::Rectify(right_lut, copy_right.data, rect_right.data, copy_right.cols,
						copy_right.rows, copy_right.channels());

				last_left = rect_left;
				last_right = rect_right;
			}
		} else {
			last_left = copy_left;
			last_right = copy_right;
		}

		compute_disparity(last_left, last_right);

		return true;
	}
	worker_shutting_down = true;
	return false;
}

void stereo_processor::save_current() {
	cv::imwrite("left.png", last_left);
	cv::imwrite("right.png", last_right);
}

/**
 * Compute disparity and emit ready frame
 * @param left
 * @param right
 */
void stereo_processor::compute_disparity(cv::Mat left, cv::Mat right) {
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
void stereo_processor::compute_disparity_daisy(cv::Mat left, cv::Mat right) {

}
#endif

int stereo_processor::get_v_offset() {
	return this->right_v_offset;
}

#if CV_VERSION_EPOCH == 2 || (!defined CV_VERSION_EPOCH && CV_VERSION_MAJOR == 2)
void stereo_processor::set_minimum_disparity(int value) {
	stereo_matcher.minDisparity = value;
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_num_disparities(int value) {
	stereo_matcher.numberOfDisparities = value - (value % 16);
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_window_size(int value) {
	stereo_matcher.SADWindowSize = value + ((value + 1) % 2);
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_p1(int value) {
	if(value < stereo_matcher.P2) {
		stereo_matcher.P1 = value;
		compute_disparity(last_left,last_right);
	}
}
void stereo_processor::set_p2(int value) {
	if(value > stereo_matcher.P1) {
		stereo_matcher.P2 = value;
		compute_disparity(last_left,last_right);
	}
}
void stereo_processor::set_pre_filter_cap(int value) {
	stereo_matcher.preFilterCap = value;
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_uniqueness_ratio(int value) {
	stereo_matcher.uniquenessRatio = value;
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_speckle_window_size(int value) {
	stereo_matcher.speckleRange = value;
	compute_disparity(last_left,last_right);
}
void stereo_processor::set_speckle_range(int value) {
	stereo_matcher.speckleRange = value;
	compute_disparity(last_left,last_right);
}
#elif CV_VERSION_MAJOR == 3
void stereo_processor::set_minimum_disparity(int value) {
	stereo_matcher->setMinDisparity(value);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_num_disparities(int value) {
	stereo_matcher->setNumDisparities(value - (value % 16));
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_window_size(int value) {
	int new_window_size = value + ((value + 1) % 2);
	stereo_matcher->setBlockSize(new_window_size);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_p1(int value) {
	if (value < stereo_matcher->getP2()) {
		stereo_matcher->setP1(value);
		compute_disparity(last_left, last_right);
	}
}
void stereo_processor::set_p2(int value) {
	if (value > stereo_matcher->getP1()) {
		stereo_matcher->setP2(value);
		compute_disparity(last_left, last_right);
	}
}
void stereo_processor::set_pre_filter_cap(int value) {
	stereo_matcher->setPreFilterCap(value);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_uniqueness_ratio(int value) {
	stereo_matcher->setUniquenessRatio(value);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_speckle_window_size(int value) {
	stereo_matcher->setSpeckleWindowSize(value);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_speckle_range(int value) {
	stereo_matcher->setSpeckleRange(value);
	compute_disparity(last_left, last_right);
}
void stereo_processor::set_v_offset(int value) {
	right_v_offset = value;
	compute_disparity(last_left, last_right);
}

#endif

} /* namespace stereo_workbench */
} /* namespace reco */
