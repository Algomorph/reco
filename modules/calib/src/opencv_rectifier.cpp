/*
 * opencv_rectifier.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/utils/debug_util.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <reco/calib/opencv_rectifier.hpp>

namespace reco {
namespace calib {

opencv_rectifier::opencv_rectifier(){}

/**
 * Loads calibration from opencv xml formatted file.
 * @param opencv_calibration_path path to the said file.
 */
opencv_rectifier::opencv_rectifier(const std::string& opencv_calibration_path,double scale_factor){
	set_calibration(opencv_calibration_path, scale_factor);
}

opencv_rectifier::~opencv_rectifier(){}


/**
 * Loads calibration from opencv xml formatted file.
 * @param path path to the said file.
 */
void opencv_rectifier::set_calibration(const std::string& path, double scale_factor){
	cv::FileStorage fs(path, cv::FileStorage::READ);
	fs["K1"] >> K1;
	fs["K2"] >> K2;
	fs["d1"] >> d1;
	fs["d2"] >> d2;
	fs["R"] >> R;
	fs["T"] >> T;

	K1 *= scale_factor;
	K2 *= scale_factor;
	K1.at<double>(2,2) = 1.0;
	K2.at<double>(2,2) = 1.0;

	cv::Size im_size(static_cast<int>(scale_factor*(int)fs["width"]),
			static_cast<int>(scale_factor*(int)fs["height"]));

	fs.release();
	compute_maps(T,R,K1,d1,K2,d2,im_size);
}

void opencv_rectifier::compute_maps(const cv::Mat& T, const cv::Mat& R,
		const cv::Mat& K1, const cv::Mat& d1,
		const cv::Mat& K2, const cv::Mat& d2,
		const cv::Size im_size){
	double factor = 1.8;
	cv::Size new_size((int)(im_size.width*factor),(int)(im_size.height*factor));
	cv::stereoRectify(K1, d1, K2, d2, im_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,-1.0,new_size);
	cv::initUndistortRectifyMap(K1, d1, R1, P1, new_size, CV_32FC1, map1x, map1y);
	cv::initUndistortRectifyMap(K2, d2, R2, P2, new_size, CV_32FC1, map2x, map2y);
}

static void report_int_mismatch(int expected, int received, const std::string& name){
	if(expected != received){
		dpt("Expected " + name + ": " << expected + ". Got: " << received);
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
		dpt("Lookup table sizes don't match image sizes. Skipping rectification.");
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

const cv::Mat& opencv_rectifier::get_left_camera_matrix() const{
	return this->K1;
};
const cv::Mat& opencv_rectifier::get_right_camera_matrix() const{
	return this->K2;
};
const cv::Mat& opencv_rectifier::get_left_rectified_camera_matrix() const{
	return this->P1;
};
const cv::Mat& opencv_rectifier::get_right_rectified_camera_matrix() const{
	return this->P2;
};

const cv::Mat& opencv_rectifier::get_translation_between_cameras() const{
	return this->T;
};
const cv::Mat& opencv_rectifier::get_rotation_between_cameras() const{
	return this->R;
};
const cv::Mat& opencv_rectifier::get_projection_matrix() const{
	return this->Q;
}

double opencv_rectifier::get_baseline() const{
	return this->Q.at<float>(3,2);
}

} /* namespace calib */
} /* namespace reco */
