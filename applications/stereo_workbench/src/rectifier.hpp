/*
 * rectifier.h
 *
 *  Created on: Oct 12, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#include <calibu/Calibu.h>

#include <opencv2/core/core.hpp>

namespace reco{
namespace stereo_workbench{

//abstract rectifier class

class rectifier{
public:
	rectifier(){}
	virtual ~rectifier(){}
	virtual void set_calibration(std::shared_ptr<calibu::Rigd> calibration) = 0;
	virtual void rectify(const cv::Mat& left,const cv::Mat& right,
				cv::Mat& rect_left, cv::Mat& rect_right) = 0;

};

}//stereo_workbench
}//reco


