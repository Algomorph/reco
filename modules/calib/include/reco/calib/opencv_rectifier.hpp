/*
 * opencv_rectifier.h
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/calib/rectifier.hpp>

namespace reco {
namespace calib {

class opencv_rectifier:
		public rectifier {
public:
	opencv_rectifier();
	opencv_rectifier(const std::string& opencv_calibration_path, double scale_factor = 1.0);
	virtual ~opencv_rectifier();
	virtual void set_calibration(const std::string& path,  double scale_factor = 1.0);
	virtual void rectify(const cv::Mat& left,const cv::Mat& right,
				cv::Mat& rect_left, cv::Mat& rect_right);
	const cv::Mat& get_left_camera_matrix() const;
	const cv::Mat& get_right_camera_matrix() const;
	const cv::Mat& get_left_rectified_camera_matrix() const;
	const cv::Mat& get_right_rectified_camera_matrix() const;
	const cv::Mat& get_translation_between_cameras() const;
	const cv::Mat& get_rotation_between_cameras() const;
	const cv::Mat& get_projection_matrix() const;
	void print() const;
	double get_baseline() const;

private:
	cv::Mat K0,K1,R,T,d0,d1;
	cv::Mat R0,R1,P0,P1,Q;
	void compute_maps(const cv::Mat& T, const cv::Mat& R,
			const cv::Mat& K1, const cv::Mat& d1,
			const cv::Mat& K2, const cv::Mat& d2,
			const cv::Size im_size);
	cv::Mat map0x,map0y,map1x,map1y;
	cv::Size im_size;

};

} /* namespace calib */
} /* namespace reco */
