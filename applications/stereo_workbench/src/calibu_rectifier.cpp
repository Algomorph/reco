/*
 * calibu_rectifier.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/calibu_rectifier.h>
#include <calibu/cam/stereo_rectify.h>
#include <reco/utils/debug_util.h>

namespace reco {
namespace stereo_workbench {

calibu_rectifier::calibu_rectifier(){}

calibu_rectifier::calibu_rectifier(std::shared_ptr<calibu::Rigd> calibration){
	if(calibration){
		set_calibration(calibration);
	}
}

void calibu_rectifier::set_calibration(std::shared_ptr<calibu::Rigd> calibration){
	Sophus::SE3d T_nr_nl;
	calibu::CreateScanlineRectifiedLookupAndCameras(
			calibration->cameras_[1]->Pose(),
			calibration->cameras_[0],
			calibration->cameras_[1],
			//T_nr_nl, left_lut, right_lut
			T_nr_nl, right_lut, left_lut
			);
	dpt("Original translation vector:");
	dpt(calibration->cameras_[1]->Pose().translation());
	dpt("Final translation vector:");
	dpt(T_nr_nl.translation());
}

static void report_int_mismatch(int expected, int received, const std::string& name){
	if(expected != received){
		dpt("Expected " + name + ": " << expected + ". Got: " << received);
	}
}

void calibu_rectifier::rectify(const cv::Mat& left, const cv::Mat& right, cv::Mat& left_rect,
		cv::Mat& right_rect){
	bool sizes_match = (int)left_lut.Width() == left.cols
				&& (int)left_lut.Height() == left.rows
				&& (int)right_lut.Width() == right.cols
				&& (int)right_lut.Height() == right.rows;
		if (!sizes_match) {
			puts("Lookup table sizes don't match image sizes. Skipping rectification.");
			report_int_mismatch(left_lut.Width(),left.cols, "left width");
			report_int_mismatch(left_lut.Height(),left.rows, "left height");
			report_int_mismatch(right_lut.Width(),right.cols, "right width");
			report_int_mismatch(right_lut.Height(),right.rows, "right height");
			left_rect = left;
			right_rect = right;
		} else {
			cv::Mat rect_left(left.rows, left.cols, left.type());
			cv::Mat rect_right(right.rows, right.cols, right.type());
			calibu::Rectify(left_lut, left.data, rect_left.data, left.cols,
					left.rows, left.channels());
			calibu::Rectify(right_lut, right.data, rect_right.data, right.cols,
					right.rows, right.channels());

			left_rect = rect_left;
			right_rect = rect_right;
		}
}

calibu_rectifier::~calibu_rectifier(){}

} /* namespace stereo_workbench */



} /* namespace reco */
