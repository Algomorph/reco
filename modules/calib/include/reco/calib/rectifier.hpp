/*
 * rectifier.h
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#include <opencv2/core/core.hpp>

namespace reco{
namespace calib{

//abstract rectifier class

class rectifier{
public:
	rectifier(){}
	virtual ~rectifier(){};
	virtual void set_calibration(const std::string& path, double scale_factor) = 0;
	virtual void rectify(const cv::Mat& left,const cv::Mat& right,
				cv::Mat& rect_left, cv::Mat& rect_right) = 0;

};

}//calib
}//reco


