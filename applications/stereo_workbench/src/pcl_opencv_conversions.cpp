/*
 * pcl_opencv_conversions.cpp
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

#include <reco/stereo_workbench/pcl_opencv_conversions.hpp>
#include <reco/utils/debug_util.h>
#include <pcl/io/io.h>

namespace reco {
namespace stereo_workbench {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(const cv::Mat& depth, const cv::Mat& K,
		Eigen::Matrix<float, 3, 3> R,
		Eigen::Vector3f T, uint8_t r, uint8_t g, uint8_t b) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	add_to_cloud(depth, K, R, T, r, g, b, *cloud);
	return cloud;
}

/**
 * \brief Convert an OpenCV depth image to an unorganized PCL point cloud using the specified camera parameters
 * \param[in] depth OpenCV depth image (CV_32F where depth is expressed in millimetres)
 * \param[in] K depth camera intrinsic matrix (CV_32F)
 * \param[in] R depth camera rotation
 * \param[in] T depth camera translation
 * \param[out] cloud PCL pointcloud
 */
void add_to_cloud(const cv::Mat& depth, const cv::Mat& K, Eigen::Matrix<float, 3, 3> R,
		Eigen::Vector3f T, uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

	if (depth.type() != CV_32F) {
		pcl::console::print_error("[add_to_cloud] depth matrix must be CV_32F");
		exit(EXIT_FAILURE);
	}
	if (K.type() != CV_32F) {
		pcl::console::print_error("[add_to_cloud] calibration matrix must be CV_32F");
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
			pt += T;
			pt = R * pt;
			pcl::PointXYZRGB ptRGB(r, g, b);
			ptRGB.x = pt.x();
			ptRGB.y = pt.y();
			ptRGB.z = pt.z();
			cloud.push_back(ptRGB);
		}
	}

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(
		const cv::Mat& disparity, const cv::Mat& mask,
		const cv::Mat& Q, Eigen::Matrix<double, 3, 3> R,
		Eigen::Vector3d T, uint8_t r, uint8_t g, uint8_t b) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	add_to_cloud(disparity, mask, Q, R, T, r, g, b, *cloud);
	return cloud;

}

/**
 * \brief Convert an OpenCV disparity image to an unorganized PCL point cloud using the specified camera parameters
 * \param[in] disparity OpenCV disparity image (CV_16U where each value is disparity in pixels)
 * \param[in] mask OpenCV disparity mask (CV_8U with non-zero values at valid disparity values)
 * \param[in] Q stereo projection matrix, CV_64F
 * \param[in] R camera (pov of disparity) rotation
 * \param[in] T camera (pov of disparity) translation
 * \param[out] cloud PCL pointcloud
 */
void add_to_cloud(
		const cv::Mat& disparity, const cv::Mat& mask,
		const cv::Mat& Q, Eigen::Matrix<double, 3, 3> R,
		Eigen::Vector3d T, uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

	if (disparity.type() != CV_16UC1) {
		pcl::console::print_error("[add_to_cloud] disparity matrix type must be CV_16U");
		exit(EXIT_FAILURE);
	}
	if (mask.type() != CV_8UC1) {
		pcl::console::print_error("[add_to_cloud] mask matrix must type be CV_8U");
		exit(EXIT_FAILURE);
	}
	if (Q.type() != CV_64F) {
		pcl::console::print_error("[add_to_cloud] calibration matrix type must be CV_64F");
		exit(EXIT_FAILURE);
	}

	const double f = Q.at<double>(2, 3);
	const double inv_baseline = Q.at<double>(3, 2);
	const double ox = Q.at<double>(0, 3);
	const double oy = Q.at<double>(1, 3);

	Eigen::Vector3d pt;

	const short* cur_disp = disparity.ptr<short>();
	uchar* cur_mask = mask.data;

	for (int row = 0; row < disparity.rows; row++) {
		for (int col = 0; col < disparity.cols; col++, cur_disp++, cur_mask++) {
			if (cur_mask) {
				float ib_d = inv_baseline * (*cur_disp);
				pt << (col - ox) / ib_d, (row - oy) / ib_d, f / ib_d;
				pt += T;
				pt = R * pt;
				pcl::PointXYZRGB ptRGB(r, g, b);
				ptRGB.x = pt.x();
				ptRGB.y = pt.y();
				ptRGB.z = pt.z();
				cloud.push_back(ptRGB);
			}
		}
	}
}

/**
 * \brief Convert an OpenCV disparity image to an unorganized PCL point cloud using the specified camera parameters
 * \param[in] disparity OpenCV disparity image (CV_16U where each value is disparity in pixels)
 * \param[in] mask OpenCV disparity mask (CV_8U with non-zero values at valid disparity values)
 * \param[in] Q stereo projection matrix, CV_64F
 * \param[out] cloud PCL pointcloud
 */
void add_to_cloud(
		const cv::Mat& disparity, const cv::Mat& mask,
		const cv::Mat& Q, uint8_t r, uint8_t g, uint8_t b,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

	if (disparity.type() != CV_16UC1) {
		pcl::console::print_error("[add_to_cloud] disparity matrix type must be CV_16U");
		exit(EXIT_FAILURE);
	}
	if (mask.type() != CV_8UC1) {
		pcl::console::print_error("[add_to_cloud] mask matrix must type be CV_8U");
		exit(EXIT_FAILURE);
	}
	if (disparity.rows != mask.rows || disparity.cols != mask.cols) {
		pcl::console::print_error(
				"[add_to_cloud] mask and disparity matrices must have the same dimensions.");
		exit(EXIT_FAILURE);
	}
	if (Q.type() != CV_64F) {
		pcl::console::print_error("[add_to_cloud] stereo projection matrix type must be CV_64F");
		exit(EXIT_FAILURE);
	}
	const float f = Q.at<float>(2, 3);
	const float inv_baseline = Q.at<float>(3, 2);
	const float ox = Q.at<float>(0, 3);
	const float oy = Q.at<float>(1, 3);

	Eigen::Vector3d pt;

	const short* cur_disp = disparity.ptr<short>();
	uchar* cur_mask = mask.data;

	for (int row = 0; row < disparity.rows; row++) {
		for (int col = 0; col < disparity.cols; col++, cur_disp++, cur_mask++) {
			if (cur_mask) {
				float ib_d = inv_baseline * (*cur_disp);
				pcl::PointXYZRGB ptRGB(r, g, b);
				ptRGB.x = (col - ox) / ib_d; //equiv. to (x-x_0) * z / f
				ptRGB.y = (row - oy) / ib_d; //equiv. to (y-y_0) * z / f
				ptRGB.z = f / ib_d; //equiv. to fB/d
				cloud.push_back(ptRGB);
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud(
		const cv::Mat& disparity, const cv::Mat& color,
		const cv::Mat& mask,
		const cv::Mat& Q) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	add_to_cloud(disparity, color, mask, Q, *cloud);
	return cloud;
}

/**
 * \brief Convert an OpenCV disparity image to an unorganized PCL point cloud using the specified camera parameters
 * \param[in] disparity OpenCV disparity image (CV_16U where each value is disparity in pixels)
 * \param[in] color OpenCV color image (CV_8UC3) corresponding to the depth image
 * \param[in] mask OpenCV disparity mask (CV_8U with non-zero values at valid disparity values)
 * \param[in] Q stereo projection matrix, CV_32F
 * \param[out] cloud PCL pointcloud
 */
void add_to_cloud(
		const cv::Mat& disparity, const cv::Mat& color, const cv::Mat& mask, const cv::Mat& Q,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

	if (disparity.type() != CV_16UC1) {
		pcl::console::print_error("[add_to_cloud] disparity matrix type must be CV_16U");
		exit(EXIT_FAILURE);
	}
	if (color.type() != CV_8UC3) {
		pcl::console::print_error("[add_to_cloud] color matrix must type be CV_8UC3");
		exit(EXIT_FAILURE);
	}
	if (mask.type() != CV_8UC1) {
		pcl::console::print_error("[add_to_cloud] mask matrix must type be CV_8U");
		exit(EXIT_FAILURE);
	}
	if (disparity.rows != mask.rows || disparity.cols != mask.cols || mask.cols != color.cols
			|| mask.rows != color.rows) {
		pcl::console::print_error(
				"[add_to_cloud] mask, color, and disparity matrices must have the same dimensions.");
		exit(EXIT_FAILURE);
	}
	if (Q.type() != CV_64F) {
		pcl::console::print_error("[add_to_cloud] stereo projection matrix type must be CV_64F");
		exit(EXIT_FAILURE);
	}

	const double f = Q.at<double>(2, 3);
	const double inv_baseline = Q.at<double>(3, 2);
	const double ox = Q.at<double>(0, 3);
	const double oy = Q.at<double>(1, 3);

	const unsigned short* cur_disp = disparity.ptr<unsigned short>();
	uchar* color_px = color.data;
	uchar* cur_mask = mask.data;

	for (int row = 0; row < disparity.rows; row++) {
		for (int col = 0; col < disparity.cols; col++, cur_disp++, cur_mask++, color_px += 3) {
			if (*cur_mask) {
				float ib_d = inv_baseline * static_cast<double>(*cur_disp) / 16;
				pcl::PointXYZRGB pt(*(color_px + 2), *(color_px + 1), *(color_px));
				pt.x = (col + ox) / ib_d; //equiv. to (x-x_0) * z / f
				pt.y = (row + oy) / ib_d;//equiv. to (y-y_0) * z / f
				pt.z = f / ib_d;//equiv. to fB/d
				cloud.push_back(pt);
			}
		}
	}
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_cloud_direct(
		const cv::Mat& vertices, const cv::Mat& color, const cv::Mat& mask) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	add_to_cloud_direct(vertices, color, mask, *cloud);
	return cloud;
}
void add_to_cloud_direct(
		const cv::Mat& vertices, const cv::Mat& color, const cv::Mat& mask,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
//	uchar* color_px = color.data;
//	const float* vertex = vertices.ptr<float>();
//	uchar* cur_mask = mask.data;
//	for (int i_pt = 0; i_pt < vertices.rows*vertices.cols; i_pt++, color_px+=3, vertex+=3, cur_mask++) {
//		pcl::PointXYZRGB pt(*(color_px + 2), *(color_px + 1), *(color_px));
//		pt.x = *vertex;
//		pt.y = *(vertex+1);
//		pt.z = *(vertex+2);
//		cloud.push_back(pt);
//	}
	for (int row = 0; row < vertices.rows; row++) {
		for (int col = 0; col < vertices.cols; col++) {
			if (mask.at<uchar>(row, col)) {
				cv::Vec3f vert = vertices.at<cv::Vec3f>(row, col);
				cv::Vec3b bgr = color.at<cv::Vec3b>(row, col);
				pcl::PointXYZRGB pt(bgr.val[2], bgr.val[1], bgr.val[0]);
				pt.x = vert.val[0];
				pt.y = vert.val[1];
				pt.z = vert.val[2];
				cloud.push_back(pt);
			}
		}
	}

}

} //stereo_workbench
} //reco

