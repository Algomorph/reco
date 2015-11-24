/*
 * opencv_rectifier.h
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once

#include <reco/stereo/rectifier.hpp>

namespace reco {
namespace stereo {

class opencv_rectifier:
		public rectifier {
public:
	opencv_rectifier();
	opencv_rectifier(const std::string& opencv_calibration_path);
	opencv_rectifier(std::shared_ptr<calibu::Rigd> calibration);
	virtual ~opencv_rectifier();
	virtual void set_calibration(std::shared_ptr<calibu::Rigd> calibration);
	void set_calibration_opencv(const std::string& path);
	virtual void rectify(const cv::Mat& left,const cv::Mat& right,
				cv::Mat& rect_left, cv::Mat& rect_right);
private:
	void compute_maps(const cv::Mat& T, const cv::Mat& R,
			const cv::Mat& K1, const cv::Mat& d1,
			const cv::Mat& K2, const cv::Mat& d2,
			const cv::Size im_size);
	cv::Mat map1x,map1y,map2x,map2y;

};

} /* namespace stereo */
} /* namespace reco */
