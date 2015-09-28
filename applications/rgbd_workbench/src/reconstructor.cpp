/*
 * reconstructor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/color_util.h>

//datapipe
#include <reco/datapipe/kinect_v2_info.h>
#include "reconstructor.h"

namespace reco {
namespace rgbd_workbench {

reconstructor::reconstructor(
		datapipe::frame_buffer_type input_buffer,
		std::shared_ptr<point_cloud_buffer> output_buffer,
		std::shared_ptr<misc::calibration_parameters> calibration
		)
	:worker(),
	 output_buffer(output_buffer),
	 calibration(calibration),
	 input_buffer(input_buffer){
	if(!calibration){
		err(std::runtime_error) << "Trying to initialize reconstruction with no calibration loaded!" << enderr;
	}
	if(!output_buffer){
		err(std::runtime_error) << "Trying to initialize reconstruction with no output_buffer loaded!" << enderr;
	}

	cloud_colors.reserve(calibration->get_num_kinects());
	//color clouds for each kinect with uniform random colors for now
	for(int i_kinect = 0; i_kinect < calibration->get_num_kinects(); i_kinect++){
		cloud_colors.push_back(utils::generate_random_color());
	}

}

reconstructor::~reconstructor(){
	stop();
	output_buffer->clear();
}

void reconstructor::pre_thread_join(){
	//run through a "fake" frame to queue to ensure thread doesn't get
	//stuck on popping from an empty queue
	std::shared_ptr<hal::ImageArray> dummy;
	input_buffer->push_back(dummy);
}



static void cvDepth32F2pclCloudColor(const cv::Mat& depth, const cv::Mat& K, Eigen::Matrix<float, 3, 3> R,
		Eigen::Vector3f T, pcl::PointCloud<pcl::PointXYZRGB>& cloud, uint32_t rgb) {
	// Check input
	if (pcl::getFieldsList(cloud).find("x y z") == std::string::npos) {
		std::cout << pcl::getFieldsList(cloud) << std::endl;
		pcl::console::print_error("[cvDepth2pclCloud] output cloud must contain xyz data");
		exit(EXIT_FAILURE);
	}
	if (K.depth() != CV_32F) {
		pcl::console::print_error("[cvDepth2pclCloud] calibration matrix must be CV_32F");
		exit(EXIT_FAILURE);
	}

	const float inv_fx = 1.0 / K.at<float>(0, 0);
	const float inv_fy = 1.0 / K.at<float>(1, 1);
	const float ox = K.at<float>(0, 2);
	const float oy = K.at<float>(1, 2);

	Eigen::Vector3f pt;
	for (int row = 0; row < depth.rows; row++) {
		for (int col = 0; col < depth.cols; col++) {
			float z = depth.at<float>(row, col) / 1000.0F;   //convert mm to m
			//equivalent to multiplying the straight-up pixel coords + depth by inverse of intrinsic matrix K
			pt << (col - ox) * z * inv_fx, (row - oy) * z * inv_fy, z;
			pt = R*pt;
			pt += T;
			pcl::PointXYZRGB ptRGB;
			ptRGB.rgb = *reinterpret_cast<float*>(&rgb);
			ptRGB.x = pt.x();
			ptRGB.y = pt.y();
			ptRGB.z = pt.z();
			cloud.push_back(ptRGB);
		}
	}

}

bool reconstructor::do_unit_of_work(){
	emit frame_consumed();

	const int num_kinects = calibration->get_num_kinects();
	const int depth_offset = datapipe::kinect_v2_info::depth_channel.offset();
	const int channels_per_kinect = datapipe::kinect_v2_info::channels.size();

	std::shared_ptr<hal::ImageArray> images = input_buffer->pop_front();
	if(!images){
		//if the frame came in as empty, time to go "bye-bye"
		return false;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i_kinect = 0; i_kinect < num_kinects; i_kinect++){
		std::shared_ptr<hal::Image> depth_img = images->at(i_kinect * channels_per_kinect + depth_offset);
		//cv::Mat cv_depth_img = *reinterpret_cast<cv::Mat*>(depth_img.get());
		cvDepth32F2pclCloudColor(*(depth_img.get()), calibration->depth_intrinsics[i_kinect],
				calibration->depth_rotations[i_kinect], calibration->depth_translations[i_kinect], *cloud,
							cloud_colors[i_kinect]);

	}
	this->output_buffer->append_point_cloud(cloud);

	emit frame_processed();
	//frame_im_arr
	return true;
}

} /* namespace workbench */
} /* namespace reco */
