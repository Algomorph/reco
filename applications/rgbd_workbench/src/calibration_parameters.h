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
namespace rgbd_workbench {


class calibration_parameters {
private:
	std::shared_ptr<calibu::Rigd> rig;
public:
	//TODO: not good to (1) keep parallel structures and (2) expose vectors
	std::vector<cv::Mat> depth_intrinsics;
	std::vector<Eigen::Matrix<float, 3, 3>> depth_rotations;
	std::vector<Eigen::Matrix<float, 3, 1>> depth_translations;

	bool empty();
	int get_num_kinects();

	calibration_parameters();
	calibration_parameters(const std::string& file_name);
	virtual ~calibration_parameters();
};

} /* namespace rgbd_workbench */
} /* namespace reco */

#endif /* RECO_WORKBENCH_CALIBRATION_PARAMETERS_H_ */
