/*
 * calibration_parameters.h
 *
 *  Created on: Sep 11, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#pragma once
#ifndef RECO_WORKBENCH_CALIBRATION_PARAMETERS_H_
#define RECO_WORKBENCH_CALIBRATION_PARAMETERS_H_

//calibu
#include <calibu/Calibu.h>
//opencv
#include <opencv2/core/core.hpp>

namespace reco {
namespace misc {


class calibration_parameters {
private:

public:
	//TODO: 708 not good to (1) keep parallel structures and (2) expose vectors
	std::vector<cv::Mat> depth_intrinsics;
	std::vector<Eigen::Matrix<float, 3, 3>> depth_rotations;
	std::vector<Eigen::Matrix<float, 3, 1>> depth_translations;
	std::shared_ptr<calibu::Rigd> rig;

	bool empty();
	int get_num_kinects();

	calibration_parameters();
	calibration_parameters(const std::string& file_name);
	virtual ~calibration_parameters();
};

} /* namespace misc */
} /* namespace reco */

#endif /* RECO_WORKBENCH_CALIBRATION_PARAMETERS_H_ */
