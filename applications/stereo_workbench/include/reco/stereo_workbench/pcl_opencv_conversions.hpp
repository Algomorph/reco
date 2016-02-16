/*
 * pcl_opencv_conversions.hpp
 *
 *  Created on: Dec 17, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#pragma once

//pcl
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cstdint>



namespace cv {
class Mat;
} /* namespace cv */

namespace reco{
namespace stereo_workbench{

#define Z_LIMIT_DEFAULT 0.6

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(const cv::Mat& depth,
		const cv::Mat& K, Eigen::Matrix<float, 3, 3> R,
		Eigen::Vector3f T,  uint8_t r, uint8_t g, uint8_t b);

void add_to_cloud(const cv::Mat& depth,
		const cv::Mat& K, Eigen::Matrix<float, 3, 3> R,
		Eigen::Vector3f T,  uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(
		const cv::Mat& disparity,const cv::Mat& mask,
		const cv::Mat& Q, Eigen::Matrix<double, 3, 3> R,
		Eigen::Vector3d T,  uint8_t r, uint8_t g, uint8_t b,
		double z_limit = Z_LIMIT_DEFAULT);

void add_to_cloud(
		const cv::Mat& disparity,const cv::Mat& mask,
		const cv::Mat& Q, Eigen::Matrix<double, 3, 3> R,
		Eigen::Vector3d T,  uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		double z_limit = Z_LIMIT_DEFAULT);

void add_to_cloud(
		const cv::Mat& disparity,const cv::Mat& mask,
		const cv::Mat& Q, uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		double z_limit = Z_LIMIT_DEFAULT);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(
		const cv::Mat& disparity, const cv::Mat& color, const cv::Mat& Q,
		cv::InputArray mask = cv::noArray(),
		double z_limit = Z_LIMIT_DEFAULT);

void add_to_cloud(
		const cv::Mat& disparity, const cv::Mat& color,
		const cv::Mat& Q, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		cv::InputArray mask = cv::noArray(), double z_limit = Z_LIMIT_DEFAULT);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud_direct(
		const cv::Mat& vertices, const cv::Mat& color, const cv::Mat& mask,
		double z_limit = Z_LIMIT_DEFAULT);
void add_to_cloud_direct(
		const cv::Mat& vertices, const cv::Mat& color, const cv::Mat& mask,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud, double z_limit = Z_LIMIT_DEFAULT);

}//stereo_workbench
}//reco


