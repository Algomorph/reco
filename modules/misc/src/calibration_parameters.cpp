/*
 * calibration_parameters.cpp
 *
 *  Created on: Sep 11, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

//local
#include <reco/datapipe/kinect_v2_info.h>

//utils
#include <reco/utils/cpp_exception_util.h>

//eigen
#include <Eigen/Dense>

//opencv
#include <opencv2/core/core_c.h>
#include <opencv2/core/eigen.hpp>

//local
#include <reco/misc/calibration_parameters.h>

namespace reco {
namespace misc {

/**
 * Default constructor, builds empty calibration parameters object
 */
calibration_parameters::calibration_parameters(){

}

/**
 * Load calibration parameters from file
 * @param file_path path to the file with the calibration parameters
 * @param num_kinects_in_pipe number of kinects in the input pipe
 */
calibration_parameters::calibration_parameters(const std::string& file_path){
	//parse intrinsics
	rig = calibu::ReadXmlRig(file_path);

	//to aviod magic numbers
	const int n_channels_per_kinect = datapipe::kinect_v2_info::channels.size();
	const int depth_offset = datapipe::kinect_v2_info::depth_channel.offset();
	const int num_kinects_in_rig =(int)rig->cameras_.size() / n_channels_per_kinect;

	for (int i_kinect = 0; i_kinect < num_kinects_in_rig; i_kinect++) {
		//pick out the depth cam matrix
		Eigen::Matrix3f cam_model = rig->cameras_[depth_offset
				+ i_kinect * n_channels_per_kinect]->K().cast<float>();
		//convert to opencv matrix
		//TODO: 920 possibly get rid of the need to convert to OpenCV
		cv::Mat K_depth(3, 3, CV_32F);
		cv::eigen2cv(cam_model, K_depth);

		//store intrinsics for future use
		depth_intrinsics.push_back(K_depth);
		depth_rotations.push_back(
				rig->cameras_[depth_offset + i_kinect * n_channels_per_kinect]->Pose().rotationMatrix().cast<
						float>());
		Eigen::Vector3f translation =
				rig->cameras_[depth_offset + i_kinect * n_channels_per_kinect]->Pose().translation().cast<
						float>().col(0);
		depth_translations.push_back(translation);
	}
}

int calibration_parameters::get_num_kinects(){
	if(empty()) return 0;
	return rig->cameras_.size() / datapipe::kinect_v2_info::channels.size();
}

/**
 * @return whether the calibration parameters are loaded or empty
 */
bool calibration_parameters::empty(){
	return this->depth_intrinsics.empty();
}

calibration_parameters::~calibration_parameters(){
}

} /* namespace misc */
} /* namespace reco */
