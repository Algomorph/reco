/*
 * opencv_rectifier.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <src/opencv_rectifier.h>
#include <reco/utils/debug_util.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace reco {
namespace stereo_workbench {

opencv_rectifier::opencv_rectifier(){}

/**
 * Loads calibration from opencv xml formatted file.
 * @param opencv_calibration_path path to the said file.
 */
opencv_rectifier::opencv_rectifier(const std::string& opencv_calibration_path){
	set_calibration_opencv(opencv_calibration_path);
}

opencv_rectifier::opencv_rectifier(std::shared_ptr<calibu::Rigd> calibration){
	set_calibration(calibration);
}

opencv_rectifier::~opencv_rectifier(){}

void opencv_rectifier::set_calibration(std::shared_ptr<calibu::Rigd> calibration){
	cv::Mat T, R, K1, K2, d1, d2;
	cv::eigen2cv(calibration->cameras_[1]->Pose().translation(),T);
	cv::eigen2cv(calibration->cameras_[1]->Pose().rotationMatrix(),R);
	cv::eigen2cv(calibration->cameras_[0]->K(),K1);
	cv::eigen2cv(calibration->cameras_[1]->K(),K2);
	Eigen::VectorXd params1 = calibration->cameras_[0]->GetParams();
	Eigen::VectorXd params2 = calibration->cameras_[1]->GetParams();
	double d1arr[5] = {params1[4],params1[5], 0.0, 0.0, params1[6]};
	double d2arr[5] = {params2[4],params2[5], 0.0, 0.0, params2[6]};
	d1 = cv::Mat(1,5,CV_64F,d1arr);
	d2 = cv::Mat(1,5,CV_64F,d2arr);
	cv::Size im_size(calibration->cameras_[0]->Width(),calibration->cameras_[0]->Height());
	compute_maps(T,R,K1,d1,K2,d2,im_size);
}

/**
 * Loads calibration from opencv xml formatted file.
 * @param path path to the said file.
 */
void opencv_rectifier::set_calibration_opencv(const std::string& path){
	cv::FileStorage fs(path, cv::FileStorage::READ);
	cv::Mat T, R, K1, K2, d1, d2;
	fs["K1"] >> K1;
	fs["K2"] >> K2;
	fs["d1"] >> d1;
	fs["d2"] >> d2;
	fs["R"] >> R;
	fs["T"] >> T;
	cv::Size im_size((int)fs["width"],(int)fs["height"]);

	fs.release();
	compute_maps(T,R,K1,d1,K2,d2,im_size);
}

void opencv_rectifier::compute_maps(const cv::Mat& T, const cv::Mat& R,
		const cv::Mat& K1, const cv::Mat& d1,
		const cv::Mat& K2, const cv::Mat& d2,
		const cv::Size im_size){
	cv::Mat R1,R2,P1,P2,Q;
	//cv::Size new_size(2880,1620);
//	cv::stereoRectify(K1, d1, K2, d2, im_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,-1.0,new_size);
//	cv::initUndistortRectifyMap(K1, d1, R1, P1, new_size, CV_32FC1, map1x, map1y);
//	cv::initUndistortRectifyMap(K2, d2, R2, P2, new_size, CV_32FC1, map2x, map2y);
	cv::stereoRectify(K1, d1, K2, d2, im_size, R, T, R1, R2, P1, P2, Q);
	cv::initUndistortRectifyMap(K1, d1, R1, P1, im_size, CV_32FC1, map1x, map1y);
	cv::initUndistortRectifyMap(K2, d2, R2, P2, im_size, CV_32FC1, map2x, map2y);
}

static void report_int_mismatch(int expected, int received, const std::string& name){
	if(expected != received){
		puts("Expected " + name + ": " << expected + ". Got: " << received);
	}
}

void opencv_rectifier::rectify(const cv::Mat& left, const cv::Mat& right, cv::Mat& left_rect,
		cv::Mat& right_rect){
	bool check_sizes = false;
	bool sizes_match = map1x.cols == left.cols
				&& map1x.rows == left.rows
				&& map2x.cols == right.cols
				&& map2x.rows == right.rows;
	if (check_sizes && !sizes_match) {
		puts("Lookup table sizes don't match image sizes. Skipping rectification.");
		report_int_mismatch( map1x.cols,left.cols, "left width");
		report_int_mismatch( map1x.rows,left.rows, "left height");
		report_int_mismatch( map2x.cols,right.cols, "right width");
		report_int_mismatch( map2x.rows,right.rows, "right height");
		left_rect = left;
		right_rect = right;
	} else {
		cv::Mat rect_left;
		cv::Mat rect_right;
		cv::remap(left, rect_left, map1x, map1y, cv::INTER_LINEAR);
		cv::remap(right, rect_right, map2x, map2y, cv::INTER_LINEAR);

		left_rect = rect_left;
		right_rect = rect_right;
	}
}

} /* namespace stereo_workbench */
} /* namespace reco */
