/*
 * opencv_rectifier.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/utils/debug_util.h>
#include <reco/utils/cpp_exception_util.h>

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <reco/calib/opencv_rectifier.hpp>

namespace reco {
namespace calib {

opencv_rectifier::opencv_rectifier():rectifier(){}

/**
 * Loads calibration from opencv xml formatted file.
 * @param opencv_calibration_path path to the said file.
 */
opencv_rectifier::opencv_rectifier(const std::string& opencv_calibration_path,double scale_factor)
	:rectifier(){
	set_calibration(opencv_calibration_path, scale_factor);
}

opencv_rectifier::~opencv_rectifier(){}


/**
 * Loads calibration from opencv xml formatted file.
 * @param path path to the said file.
 */
void opencv_rectifier::set_calibration(const std::string& path, double scale_factor){
	cv::FileStorage fs(path, cv::FileStorage::READ);

	cv::FileNode stereo_calib_node = fs["StereoCalibrationInfo"];
	cv::FileNode cameras_node = stereo_calib_node["Cameras"];
	cv::FileNode camera_0_node = cameras_node[0];
	cv::FileNode camera_1_node = cameras_node[1];

	camera_0_node["intrinsic_mat"] >> K0;
	camera_0_node["distortion_coeffs"] >> d0;
	camera_1_node["intrinsic_mat"] >> K1;
	camera_1_node["distortion_coeffs"] >> d1;
	stereo_calib_node["rotation"] >> R;
	stereo_calib_node["translation"] >> T;

	K0 *= scale_factor;
	K1 *= scale_factor;

	K0.at<double>(2,2) = 1.0;
	K1.at<double>(2,2) = 1.0;

	im_size = cv::Size(static_cast<int>(scale_factor*static_cast<int>(camera_0_node["resolution"]["width"])),
		static_cast<int>(scale_factor*static_cast<int>(camera_0_node["resolution"]["height"])));

	fs.release();
	compute_maps(T,R,K0,d0,K1,d1,im_size);
}

void opencv_rectifier::compute_maps(const cv::Mat& T, const cv::Mat& R,
		const cv::Mat& K0, const cv::Mat& d0,
		const cv::Mat& K1, const cv::Mat& d1,
		const cv::Size im_size){
	double factor = 1.8;//TODO: make this at least a class-wide constant, better yet specifiable at construction
	cv::Size new_size((int)(im_size.width*factor),(int)(im_size.height*factor));
	cv::stereoRectify(K0, d0, K1, d1, im_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY,-1.0,new_size);
	cv::initUndistortRectifyMap(K0, d0, R0, P0, new_size, CV_32FC1, map0x, map0y);
	cv::initUndistortRectifyMap(K1, d1, R1, P1, new_size, CV_32FC1, map1x, map1y);
}

static void report_int_mismatch(int expected, int received, const std::string& name){
	if(expected != received){
		dpt("Expected " + name + ": " << expected + ". Got: " << received);
	}
}

void opencv_rectifier::rectify(const cv::Mat& left, const cv::Mat& right, cv::Mat& left_rect,
		cv::Mat& right_rect){
	bool check_sizes = false;
	bool sizes_match = map0x.cols == left.cols
				&& map0x.rows == left.rows
				&& map1x.cols == right.cols
				&& map1x.rows == right.rows;
	if (check_sizes && !sizes_match) {
		dpt("Lookup table sizes don't match image sizes. Skipping rectification.");
		report_int_mismatch( map0x.cols,left.cols, "left width");
		report_int_mismatch( map0x.rows,left.rows, "left height");
		report_int_mismatch( map1x.cols,right.cols, "right width");
		report_int_mismatch( map1x.rows,right.rows, "right height");
		left_rect = left;
		right_rect = right;
	} else {
		cv::Mat rect_left;
		cv::Mat rect_right;
		cv::remap(left, rect_left, map0x, map0y, cv::INTER_LINEAR);
		cv::remap(right, rect_right, map1x, map1y, cv::INTER_LINEAR);

		left_rect = rect_left;
		right_rect = rect_right;
	}
}

void opencv_rectifier::print() const{
	std::cout << "K0:" << std::endl << K0 << std::endl;
	std::cout << "K1:" << std::endl << K1 << std::endl;
	std::cout << "d0:" << std::endl << d0 << std::endl;
	std::cout << "d1:" << std::endl << d1 << std::endl;
	std::cout << "T:" << std::endl << T << std::endl;
	std::cout << "R:" << std::endl << R << std::endl;
	std::cout << "Size: " << im_size << std::endl;
}

const cv::Mat& opencv_rectifier::get_left_camera_matrix() const{
	return this->K0;
};
const cv::Mat& opencv_rectifier::get_right_camera_matrix() const{
	return this->K1;
};
const cv::Mat& opencv_rectifier::get_left_rectified_camera_matrix() const{
	return this->P0;
};
const cv::Mat& opencv_rectifier::get_right_rectified_camera_matrix() const{
	return this->P1;
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
